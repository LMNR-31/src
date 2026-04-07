// supervisor_T.cpp — ROS 2 supervisor node.
//
// State machine:
//   INIT                → Forks takeoff process on startup.
//   TAKING_OFF          → Waits (via WNOHANG poll) for the takeoff process to exit.
//                         On exit code 0 → transitions to RUN_YAW.
//                         On any other exit → transitions directly to WAIT_TRAJ.
//   RUN_YAW             → Forks drone_yaw_360 with yaw_target_delta:=6.2831853,
//                         waits for it to exit, then transitions to WAIT_TRAJ.
//   WAIT_TRAJ           → Monitors /trajectory_progress and /trajectory_finished;
//                         transitions to WAIT_BEFORE_MISSION when a trajectory completes.
//   WAIT_BEFORE_MISSION → Waits wait_after_traj_done_s seconds (default 5.0) before
//                         launching the next mission; then transitions to RUN_MISSION.
//   RUN_MISSION         → Forks missao_P_T (or pouso with use_current_xy:=true when
//                         use_origin_as_base=true and the UAV is stably at the origin),
//                         waits for it to exit, then loops back to WAIT_TRAJ for the
//                         next cycle (infinite loop).
//
// Trajectory-detection logic (WAIT_TRAJ only):
//   • /trajectory_progress < 99.9  → new trajectory in progress (reset guards)
//   • /trajectory_progress >= 100  → trajectory done
//   • /trajectory_finished == false → new trajectory started (reset guards)
//   • /trajectory_finished == true  → trajectory done
//     (99.9 threshold avoids spurious resets from float jitter near 100.0)
//
// Stale-signal protection (post_reset_ticks_):
//   When transitioning from TAKING_OFF or RUN_MISSION to WAIT_TRAJ the
//   subscription queues may still hold trajectory signals that were published
//   by the pouso sub-process while the supervisor was in RUN_MISSION.
//   post_reset_ticks_ is set to POST_RESET_COOLDOWN_TICKS on every such
//   transition.  check_trajectory() discards any signals that arrive during
//   those ticks, preventing a spurious re-launch.
//
// All child processes are spawned via fork()/execlp() and reaped with
// waitpid(WNOHANG) inside the FSM timer to keep the executor unblocked.
//
// Parameters:
//   uav_name                (string, default "uav1")   — UAV namespace prefix
//   use_origin_as_base      (bool,   default true)     — when true (default), launch
//                             pouso with use_current_xy:=true instead of missao_P_T
//                             if the drone is within base_tol_m of (0,0) at trajectory
//                             completion and has remained there for base_hold_s seconds;
//                             when false, always launch missao_P_T.
//   wait_after_traj_done_s  (double, default 5.0)      — seconds to wait in
//                             WAIT_BEFORE_MISSION before launching the next mission
//                             after a trajectory is considered complete.
//   min_relaunch_dist_m     (double, default 0.5)      — minimum XY distance (m) the
//                             drone must have moved from the last mission launch
//                             position before a new mission is allowed; applies to
//                             both missao_P_T and pouso (local) launches; if the drone
//                             is closer than this, the mission is skipped and the
//                             supervisor returns to WAIT_TRAJ; set to 0.0 to disable.
//   base_tol_m              (double, default 0.20)     — radial tolerance (m) used to
//                             decide whether the UAV is at the origin; replaces the
//                             old hardcoded per-axis 0.1 m constant.
//   base_hold_s             (double, default 2.0)      — the UAV must remain within
//                             base_tol_m of the origin continuously for this many
//                             seconds before the local pouso is triggered; set to 0.0
//                             to trigger immediately (old behaviour).
//   pouso_xy_hold_tol       (double, default 0.10)     — xy_hold_tol passed to pouso
//                             when landing at the origin.
//   pouso_xy_hold_stable_s  (double, default 1.0)      — xy_hold_stable_s passed to
//                             pouso when landing at the origin.
//   pouso_xy_abort_tol      (double, default 0.5)      — xy_abort_tol passed to pouso
//                             when landing at the origin.
//   pouso_approach_z        (double, default -1.0)     — approach_z passed to pouso
//                             when landing at the origin; -1.0 means use current odom Z.
//
// Usage:
//   ros2 run drone_control supervisor_T

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <cstdio>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

// ── State machine states ───────────────────────────────────────────────────
enum class SupervisorState {
    INIT,                  // Initial state: will fire takeoff immediately
    TAKING_OFF,            // Takeoff process is running; waiting for it to exit
    RUN_YAW,               // drone_yaw_360 is running; waiting for it to exit
    WAIT_TRAJ,             // Waiting for a trajectory to start and then complete
    WAIT_BEFORE_MISSION,   // Trajectory done; waiting wait_after_traj_done_s before launching
    RUN_MISSION,           // missao_P_T is running; waiting for it to exit
};

// Number of FSM ticks (× 500 ms each = 1 s total) to ignore trajectory
// signals after entering WAIT_TRAJ.  This discards any stale messages that
// were queued while the supervisor was in RUN_MISSION (e.g. signals generated
// by missao_P_T's pouso sub-trajectory) before the supervisor starts
// listening for the next real external trajectory.
static constexpr int POST_RESET_COOLDOWN_TICKS = 2;

class SupervisorTNode : public rclcpp::Node {
public:
    SupervisorTNode()
    : Node("supervisor_T"),
      state_(SupervisorState::INIT),
      child_pid_(-1),
      trajectory_active_(false),
      trajectory_done_(false),
      post_reset_ticks_(0),
      odom_received_(false),
      current_x_(0.0),
      current_y_(0.0),
      use_origin_as_base_(false),
      wait_after_traj_done_s_(0.0),
      wait_start_time_(0, 0, RCL_ROS_TIME),  // sentinel; overwritten in check_trajectory()
      min_relaunch_dist_m_(0.5),
      last_mission_valid_(false),
      last_mission_x_(0.0),
      last_mission_y_(0.0),
      current_child_exec_("missao_P_T"),
      base_tol_m_(0.20),
      base_hold_s_(2.0),
      base_zone_entered_(false),
      base_zone_enter_time_(0, 0, RCL_ROS_TIME),
      pouso_xy_hold_tol_(0.10),
      pouso_xy_hold_stable_s_(1.0),
      pouso_xy_abort_tol_(0.5),
      pouso_approach_z_(-1.0)
    {
        this->declare_parameter<std::string>("uav_name", "uav1");
        uav_name_ = this->get_parameter("uav_name").as_string();

        this->declare_parameter<bool>("use_origin_as_base", true);
        use_origin_as_base_ = this->get_parameter("use_origin_as_base").as_bool();

        this->declare_parameter<double>("wait_after_traj_done_s", 5.0);
        wait_after_traj_done_s_ = this->get_parameter("wait_after_traj_done_s").as_double();

        this->declare_parameter<double>("min_relaunch_dist_m", 0.5);
        min_relaunch_dist_m_ = this->get_parameter("min_relaunch_dist_m").as_double();

        this->declare_parameter<double>("base_tol_m", 0.20);
        base_tol_m_ = this->get_parameter("base_tol_m").as_double();

        this->declare_parameter<double>("base_hold_s", 2.0);
        base_hold_s_ = this->get_parameter("base_hold_s").as_double();

        this->declare_parameter<double>("pouso_xy_hold_tol", 0.10);
        pouso_xy_hold_tol_ = this->get_parameter("pouso_xy_hold_tol").as_double();

        this->declare_parameter<double>("pouso_xy_hold_stable_s", 1.0);
        pouso_xy_hold_stable_s_ = this->get_parameter("pouso_xy_hold_stable_s").as_double();

        this->declare_parameter<double>("pouso_xy_abort_tol", 0.5);
        pouso_xy_abort_tol_ = this->get_parameter("pouso_xy_abort_tol").as_double();

        this->declare_parameter<double>("pouso_approach_z", -1.0);
        pouso_approach_z_ = this->get_parameter("pouso_approach_z").as_double();

        progress_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/trajectory_progress", 10,
            std::bind(&SupervisorTNode::progress_callback, this, std::placeholders::_1));

        finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/trajectory_finished", 10,
            std::bind(&SupervisorTNode::finished_callback, this, std::placeholders::_1));

        mission_cycle_done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/mission_cycle_done", 10);

        yaw_scan_done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/yaw_scan_done", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + uav_name_ + "/mavros/local_position/odom", 10,
            std::bind(&SupervisorTNode::odom_callback, this, std::placeholders::_1));

        // Main FSM timer — runs every 500 ms, never blocks the executor.
        fsm_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SupervisorTNode::fsm_tick, this));

        RCLCPP_INFO(this->get_logger(),
            "supervisor_T iniciado — executando takeoff automático e iniciando "
            "missao_P_T em ciclos infinitos após cada trajetória. "
            "use_origin_as_base=%s, wait_after_traj_done_s=%.1f, "
            "min_relaunch_dist_m=%.2f, base_tol_m=%.2f, base_hold_s=%.1f, "
            "pouso_xy_hold_tol=%.2f, pouso_xy_hold_stable_s=%.1f, "
            "pouso_xy_abort_tol=%.2f, pouso_approach_z=%.2f",
            use_origin_as_base_ ? "true" : "false",
            wait_after_traj_done_s_,
            min_relaunch_dist_m_,
            base_tol_m_,
            base_hold_s_,
            pouso_xy_hold_tol_,
            pouso_xy_hold_stable_s_,
            pouso_xy_abort_tol_,
            pouso_approach_z_);
    }

private:
    // ── /mavros/local_position/odom callback ──────────────────────────────
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        odom_received_ = true;

        // Track continuous time within base_tol_m_ of the origin for base_hold_s check.
        if (use_origin_as_base_) {
            const double dist = std::hypot(current_x_, current_y_);
            if (dist <= base_tol_m_) {
                if (!base_zone_entered_) {
                    base_zone_entered_    = true;
                    base_zone_enter_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(),
                        "[ODOM] 🏠 UAV entrou na zona base "
                        "(dist=%.3f m ≤ tol=%.2f m). "
                        "Aguardando estabilidade de %.1f s…",
                        dist, base_tol_m_, base_hold_s_);
                }
            } else {
                if (base_zone_entered_) {
                    base_zone_entered_ = false;
                    RCLCPP_DEBUG(this->get_logger(),
                        "[ODOM] UAV saiu da zona base "
                        "(dist=%.3f m > tol=%.2f m). "
                        "Reiniciando contagem de estabilidade.",
                        dist, base_tol_m_);
                }
            }
        }
    }

    // ── /trajectory_progress callback (active only in WAIT_TRAJ) ──────────
    void progress_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (state_ != SupervisorState::WAIT_TRAJ) {
            return;
        }
        if (msg->data < 99.9f) {
            // Trajectory is in progress; set active and reset done guard.
            if (!trajectory_active_) {
                trajectory_active_ = true;
                trajectory_done_   = false;
                RCLCPP_INFO(this->get_logger(),
                    "▶️  [WAIT_TRAJ] Nova trajetória detectada (%.1f%%) — aguardando conclusão.",
                    msg->data);
            }
        } else {
            // Progress >= 100 → trajectory finished.
            if (!trajectory_done_) {
                trajectory_active_ = true;   // ensure guard is consistent
                trajectory_done_   = true;
                RCLCPP_INFO(this->get_logger(),
                    "📊 [WAIT_TRAJ] Progresso %.1f%% — trajetória concluída.", msg->data);
            }
        }
    }

    // ── /trajectory_finished callback (active only in WAIT_TRAJ) ──────────
    void finished_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (state_ != SupervisorState::WAIT_TRAJ) {
            return;
        }
        if (!msg->data) {
            // false → a new trajectory is starting; reset done guard.
            if (!trajectory_active_) {
                trajectory_active_ = true;
                trajectory_done_   = false;
                RCLCPP_INFO(this->get_logger(),
                    "▶️  [WAIT_TRAJ] Nova trajetória detectada (/trajectory_finished=false) — "
                    "aguardando conclusão.");
            }
        } else {
            // true → trajectory finished.
            if (!trajectory_done_) {
                trajectory_active_ = true;   // ensure guard is consistent
                trajectory_done_   = true;
                RCLCPP_INFO(this->get_logger(),
                    "📨 [WAIT_TRAJ] /trajectory_finished=true — trajetória concluída.");
            }
        }
    }

    // ── Main FSM tick (every 500 ms) ───────────────────────────────────────
    void fsm_tick() {
        switch (state_) {
            case SupervisorState::INIT:
                do_init();
                break;
            case SupervisorState::TAKING_OFF:
                check_takeoff();
                break;
            case SupervisorState::RUN_YAW:
                check_yaw();
                break;
            case SupervisorState::WAIT_TRAJ:
                check_trajectory();
                break;
            case SupervisorState::WAIT_BEFORE_MISSION:
                check_wait_before_mission();
                break;
            case SupervisorState::RUN_MISSION:
                check_mission();
                break;
        }
    }

    // ── INIT: fork takeoff and transition to TAKING_OFF ───────────────────
    void do_init() {
        RCLCPP_INFO(this->get_logger(),
            "[INIT] 🛫 Disparando takeoff automático (ros2 run drone_control takeoff)…");
        child_pid_ = fork_exec("takeoff");
        if (child_pid_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                "[INIT] takeoff iniciado (PID %d). Aguardando conclusão…",
                static_cast<int>(child_pid_));
            state_ = SupervisorState::TAKING_OFF;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "[INIT] fork() falhou ao iniciar takeoff! Tentando novamente em 500 ms…");
        }
    }

    // ── TAKING_OFF: poll until takeoff process exits ───────────────────────
    void check_takeoff() {
        int status = 0;
        pid_t result = waitpid(child_pid_, &status, WNOHANG);
        if (result == 0) {
            // Still running.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[TAKING_OFF] Aguardando takeoff (PID %d)…",
                static_cast<int>(child_pid_));
            return;
        }
        if (result > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[TAKING_OFF] ✅ Takeoff concluído com sucesso. "
                "Iniciando drone_yaw_360…");
            pid_t yaw_pid = fork_exec_yaw360();
            child_pid_ = -1;   // takeoff already reaped; clear stale PID
            if (yaw_pid > 0) {
                child_pid_ = yaw_pid;
                RCLCPP_INFO(this->get_logger(),
                    "[RUN_YAW] drone_yaw_360 iniciado (PID %d) com "
                    "yaw_target_delta=6.2831853.",
                    static_cast<int>(child_pid_));
                state_ = SupervisorState::RUN_YAW;
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "[TAKING_OFF] fork() falhou ao iniciar drone_yaw_360! "
                    "Passando direto para WAIT_TRAJ…");
                reset_trajectory_guards();
                post_reset_ticks_ = POST_RESET_COOLDOWN_TICKS;
                state_ = SupervisorState::WAIT_TRAJ;
                RCLCPP_INFO(this->get_logger(),
                    "[WAIT_TRAJ] Monitorando /trajectory_progress e "
                    "/trajectory_finished…");
            }
        } else {
            RCLCPP_WARN(this->get_logger(),
                "[TAKING_OFF] ⚠️  Takeoff encerrou (código=%d). Continuando para WAIT_TRAJ.",
                WIFEXITED(status) ? WEXITSTATUS(status) : -1);
            child_pid_ = -1;
            reset_trajectory_guards();
            post_reset_ticks_ = POST_RESET_COOLDOWN_TICKS;
            state_ = SupervisorState::WAIT_TRAJ;
            RCLCPP_INFO(this->get_logger(),
                "[WAIT_TRAJ] Monitorando /trajectory_progress e /trajectory_finished…");
        }
    }

    // ── RUN_YAW: poll until drone_yaw_360 exits, then go to WAIT_TRAJ ─────
    void check_yaw() {
        int status = 0;
        pid_t result = waitpid(child_pid_, &status, WNOHANG);
        if (result == 0) {
            // Still running.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RUN_YAW] Aguardando drone_yaw_360 (PID %d)…",
                static_cast<int>(child_pid_));
            return;
        }
        if (result > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[RUN_YAW] ✅ drone_yaw_360 (PID %d) finalizado com sucesso.",
                static_cast<int>(child_pid_));
        } else if (result > 0 && WIFEXITED(status)) {
            RCLCPP_WARN(this->get_logger(),
                "[RUN_YAW] ⚠️  drone_yaw_360 (PID %d) encerrou com código %d.",
                static_cast<int>(child_pid_), WEXITSTATUS(status));
        } else {
            RCLCPP_WARN(this->get_logger(),
                "[RUN_YAW] ⚠️  drone_yaw_360 (PID %d) encerrou de forma anormal.",
                static_cast<int>(child_pid_));
        }
        child_pid_ = -1;
        reset_trajectory_guards();
        post_reset_ticks_ = POST_RESET_COOLDOWN_TICKS;

        // Signal the Python waypoint supervisor that the initial 360° scan is done.
        {
            std_msgs::msg::Bool scan_done;
            scan_done.data = true;
            yaw_scan_done_pub_->publish(scan_done);
            RCLCPP_INFO(this->get_logger(),
                "[RUN_YAW] 📢 /yaw_scan_done=true publicado.");
        }

        state_ = SupervisorState::WAIT_TRAJ;
        RCLCPP_INFO(this->get_logger(),
            "[WAIT_TRAJ] Monitorando /trajectory_progress e /trajectory_finished…");
    }

    // ── WAIT_TRAJ: wait for a complete trajectory, then launch mission ─────
    void check_trajectory() {
        // Discard any stale signals that were queued while in RUN_MISSION
        // (e.g. trajectory_finished=true from missao_P_T's pouso sub-process).
        if (post_reset_ticks_ > 0) {
            --post_reset_ticks_;
            if (trajectory_active_ || trajectory_done_) {
                RCLCPP_INFO(this->get_logger(),
                    "[WAIT_TRAJ] Sinais residuais de missao_P_T descartados "
                    "(cooldown: %d tick(s) restante(s)).",
                    post_reset_ticks_);
                reset_trajectory_guards();
            }
            return;
        }
        if (!trajectory_done_) {
            return;
        }

        // Trajectory is complete.  Log and enter the pre-mission wait state
        // instead of launching immediately.
        RCLCPP_INFO(this->get_logger(),
            "[WAIT_TRAJ] 🏁 Trajetória concluída. "
            "Aguardando %.1f s antes de iniciar nova missão…",
            wait_after_traj_done_s_);

        reset_trajectory_guards();
        wait_start_time_ = this->now();
        state_ = SupervisorState::WAIT_BEFORE_MISSION;
    }

    // ── WAIT_BEFORE_MISSION: hold wait_after_traj_done_s_ seconds, then launch ──
    void check_wait_before_mission() {
        const double elapsed = (this->now() - wait_start_time_).seconds();
        if (elapsed < wait_after_traj_done_s_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[WAIT_BEFORE_MISSION] Aguardando %.1f s antes de iniciar nova missão "
                "(%.1f s / %.1f s)…",
                wait_after_traj_done_s_, elapsed, wait_after_traj_done_s_);
            return;
        }

        // Guard: do not allow a new mission if the drone hasn't moved far enough
        // from the last mission launch position (prevents landing twice in a row
        // at the same spot).
        if (min_relaunch_dist_m_ > 0.0 && last_mission_valid_ && odom_received_) {
            const double d = std::hypot(current_x_ - last_mission_x_,
                                        current_y_ - last_mission_y_);
            if (d < min_relaunch_dist_m_) {
                RCLCPP_WARN(this->get_logger(),
                    "[WAIT_BEFORE_MISSION] ⛔ Posição atual (%.2f, %.2f) está a "
                    "%.2fm da última missão (%.2f, %.2f) — "
                    "mínimo: %.2fm. Missão ignorada para evitar pouso repetido. "
                    "Retornando a WAIT_TRAJ…",
                    current_x_, current_y_, d,
                    last_mission_x_, last_mission_y_,
                    min_relaunch_dist_m_);
                // trajectory guards were already cleared on entry to this state;
                // only set the cooldown and return to WAIT_TRAJ.
                post_reset_ticks_ = POST_RESET_COOLDOWN_TICKS;
                state_ = SupervisorState::WAIT_TRAJ;
                return;
            }
        }

        // Delay elapsed — determine which executable to launch next.
        // When use_origin_as_base_=true (default) and the drone has been within
        // base_tol_m_ of the origin continuously for base_hold_s_ seconds,
        // launch pouso with use_current_xy:=true (local position).
        // Otherwise (use_origin_as_base_=false, drone not at origin, or not yet
        // stable), run missao_P_T.
        bool at_base = false;
        if (use_origin_as_base_ && odom_received_) {
            if (base_zone_entered_) {
                const double stable_s = (this->now() - base_zone_enter_time_).seconds();
                const double dist     = std::hypot(current_x_, current_y_);
                if (stable_s >= base_hold_s_) {
                    at_base = true;
                    RCLCPP_INFO(this->get_logger(),
                        "[WAIT_BEFORE_MISSION] 🏠 UAV na zona base há %.1f s "
                        "(dist=%.3f m ≤ tol=%.2f m) — pouso local autorizado.",
                        stable_s, dist, base_tol_m_);
                } else {
                    // Still accumulating hold time — keep waiting in this state.
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[WAIT_BEFORE_MISSION] 🏠 UAV na zona base "
                        "(dist=%.3f m). Estabilidade: %.1f s / %.1f s…",
                        dist, stable_s, base_hold_s_);
                    return;
                }
            }
        }

        const char * next_exec = at_base ? "pouso (use_current_xy)" : "missao_P_T";
        current_child_exec_ = next_exec;

        if (at_base) {
            RCLCPP_INFO(this->get_logger(),
                "[WAIT_BEFORE_MISSION] ⏱️  %.1f s concluídos. UAV no ponto base "
                "(x=%.2f y=%.2f) — lançando pouso com posição local "
                "(xy_hold_tol=%.2f, xy_hold_stable_s=%.1f, "
                "xy_abort_tol=%.2f, approach_z=%.2f)…",
                wait_after_traj_done_s_, current_x_, current_y_,
                pouso_xy_hold_tol_, pouso_xy_hold_stable_s_,
                pouso_xy_abort_tol_, pouso_approach_z_);
        } else if (!odom_received_) {
            RCLCPP_WARN(this->get_logger(),
                "[WAIT_BEFORE_MISSION] ⏱️  %.1f s concluídos (sem odometria) — "
                "lançando missao_P_T…",
                wait_after_traj_done_s_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[WAIT_BEFORE_MISSION] ⏱️  %.1f s concluídos (x=%.2f y=%.2f) — "
                "lançando missao_P_T…",
                wait_after_traj_done_s_, current_x_, current_y_);
        }

        child_pid_ = at_base ? fork_exec_pouso_local() : fork_exec("missao_P_T");
        if (child_pid_ > 0) {
            // Record this launch position so the next cycle can enforce the
            // same-spot guard.
            if (odom_received_) {
                last_mission_x_     = current_x_;
                last_mission_y_     = current_y_;
                last_mission_valid_ = true;
            }
            state_ = SupervisorState::RUN_MISSION;
            RCLCPP_INFO(this->get_logger(),
                "[RUN_MISSION] %s iniciado (PID %d).",
                next_exec, static_cast<int>(child_pid_));
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "[WAIT_BEFORE_MISSION] fork() falhou ao iniciar %s! "
                "Tentando novamente em 500 ms…",
                next_exec);
            // Keep state as WAIT_BEFORE_MISSION so the next tick retries.
        }
    }

    // ── RUN_MISSION: poll until missao_P_T exits, then loop ───────────────
    void check_mission() {
        int status = 0;
        pid_t result = waitpid(child_pid_, &status, WNOHANG);
        if (result == 0) {
            // Still running.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RUN_MISSION] Aguardando %s (PID %d)…",
                current_child_exec_.c_str(), static_cast<int>(child_pid_));
            return;
        }
        if (result > 0 && WIFEXITED(status)) {
            RCLCPP_INFO(this->get_logger(),
                "[RUN_MISSION] ✅ %s (PID %d) finalizado com código %d.",
                current_child_exec_.c_str(), static_cast<int>(child_pid_),
                WEXITSTATUS(status));
        } else {
            RCLCPP_WARN(this->get_logger(),
                "[RUN_MISSION] ⚠️  %s (PID %d) encerrou de forma anormal.",
                current_child_exec_.c_str(), static_cast<int>(child_pid_));
        }
        child_pid_ = -1;

        // Signal the Python waypoint supervisor that this mission cycle is done.
        std_msgs::msg::Bool done_msg;
        done_msg.data = true;
        mission_cycle_done_pub_->publish(done_msg);
        RCLCPP_INFO(this->get_logger(),
            "[RUN_MISSION] 📢 /mission_cycle_done=true publicado.");

        reset_trajectory_guards();
        post_reset_ticks_ = POST_RESET_COOLDOWN_TICKS;
        state_ = SupervisorState::WAIT_TRAJ;
        RCLCPP_INFO(this->get_logger(),
            "[WAIT_TRAJ] %s concluído. Aguardando próxima trajetória…",
            current_child_exec_.c_str());
    }

    // ── Helpers ────────────────────────────────────────────────────────────
    pid_t fork_exec(const char * executable) {
        pid_t pid = fork();
        if (pid == 0) {
            // Child process: exec the target node.
            execlp("ros2", "ros2", "run", "drone_control", executable,
                   static_cast<char *>(nullptr));
            // execlp only returns on failure.
            _exit(1);
        }
        return pid;   // > 0 in parent on success; -1 on fork failure
    }

    // Launches: ros2 run drone_control drone_yaw_360
    //              --ros-args -p yaw_target_delta:=6.2831853
    pid_t fork_exec_yaw360() {
        pid_t pid = fork();
        if (pid == 0) {
            execlp("ros2", "ros2", "run", "drone_control", "drone_yaw_360",
                   "--ros-args", "-p", "yaw_target_delta:=6.2831853",
                   static_cast<char *>(nullptr));
            _exit(1);
        }
        return pid;   // > 0 in parent on success; -1 on fork failure
    }

    // Launches: ros2 run drone_control pouso
    //              --ros-args -p use_current_xy:=true
    //                         -p xy_hold_tol:=<pouso_xy_hold_tol_>
    //                         -p xy_hold_stable_s:=<pouso_xy_hold_stable_s_>
    //                         -p xy_abort_tol:=<pouso_xy_abort_tol_>
    //                         -p approach_z:=<pouso_approach_z_>
    // Used when the drone has returned to the origin so that it lands at the
    // current local position instead of running the full missao_P_T sequence.
    // The extra parameters allow the supervisor to tighten centering tolerances
    // for a more precise landing at the origin.
    pid_t fork_exec_pouso_local() {
        // Build parameter strings on the stack before fork() so the child
        // process inherits valid copies via the copy-on-write page tables.
        // 64 bytes is more than sufficient for any representable double value
        // (e.g. "xy_hold_stable_s:=-1234.5678" ≈ 29 chars), but we verify
        // snprintf succeeded to catch unexpected truncation in future edits.
        char xy_hold_tol_arg[64];
        char xy_hold_stable_s_arg[64];
        char xy_abort_tol_arg[64];
        char approach_z_arg[64];
        const int r1 = snprintf(xy_hold_tol_arg,      sizeof(xy_hold_tol_arg),
                                "xy_hold_tol:=%.4f",       pouso_xy_hold_tol_);
        const int r2 = snprintf(xy_hold_stable_s_arg, sizeof(xy_hold_stable_s_arg),
                                "xy_hold_stable_s:=%.4f",  pouso_xy_hold_stable_s_);
        const int r3 = snprintf(xy_abort_tol_arg,     sizeof(xy_abort_tol_arg),
                                "xy_abort_tol:=%.4f",      pouso_xy_abort_tol_);
        const int r4 = snprintf(approach_z_arg,       sizeof(approach_z_arg),
                                "approach_z:=%.4f",        pouso_approach_z_);

        if (r1 < 0 || r1 >= static_cast<int>(sizeof(xy_hold_tol_arg)) ||
            r2 < 0 || r2 >= static_cast<int>(sizeof(xy_hold_stable_s_arg)) ||
            r3 < 0 || r3 >= static_cast<int>(sizeof(xy_abort_tol_arg)) ||
            r4 < 0 || r4 >= static_cast<int>(sizeof(approach_z_arg))) {
            RCLCPP_WARN(this->get_logger(),
                "[fork_exec_pouso_local] snprintf falhou ou truncou argumento de "
                "parâmetro — sobrescrevendo com valores padrão seguros.");
            snprintf(xy_hold_tol_arg,      sizeof(xy_hold_tol_arg),
                     "xy_hold_tol:=0.1000");
            snprintf(xy_hold_stable_s_arg, sizeof(xy_hold_stable_s_arg),
                     "xy_hold_stable_s:=1.0000");
            snprintf(xy_abort_tol_arg,     sizeof(xy_abort_tol_arg),
                     "xy_abort_tol:=0.5000");
            snprintf(approach_z_arg,       sizeof(approach_z_arg),
                     "approach_z:=-1.0000");
        }

        pid_t pid = fork();
        if (pid == 0) {
            execlp("ros2", "ros2", "run", "drone_control", "pouso",
                   "--ros-args",
                   "-p", "use_current_xy:=true",
                   "-p", xy_hold_tol_arg,
                   "-p", xy_hold_stable_s_arg,
                   "-p", xy_abort_tol_arg,
                   "-p", approach_z_arg,
                   static_cast<char *>(nullptr));
            _exit(1);
        }
        return pid;   // > 0 in parent on success; -1 on fork failure
    }

    void reset_trajectory_guards() {
        trajectory_active_ = false;
        trajectory_done_   = false;
    }

    // ── Members ────────────────────────────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  progress_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     finished_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        mission_cycle_done_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        yaw_scan_done_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr                             fsm_timer_;

    SupervisorState state_;
    pid_t           child_pid_;           // PID of current child process (-1 if none)
    bool            trajectory_active_;   // true while a trajectory is in progress
    bool            trajectory_done_;     // true when trajectory completion confirmed
    int             post_reset_ticks_;    // cooldown ticks remaining after entering WAIT_TRAJ

    // Odometry / base-point detection
    std::string uav_name_;
    bool        odom_received_;
    double      current_x_;
    double      current_y_;

    // Mission launch policy
    bool        use_origin_as_base_;      // param: gate pouso on proximity to origin
    double      wait_after_traj_done_s_;  // param: seconds to wait before launching next mission
    rclcpp::Time wait_start_time_;        // time when WAIT_BEFORE_MISSION began
    double      min_relaunch_dist_m_;     // param: min distance from last launch to allow new mission
    bool        last_mission_valid_;      // true once a mission has been launched with valid odom
    double      last_mission_x_;          // odom X at the last mission launch
    double      last_mission_y_;          // odom Y at the last mission launch
    std::string current_child_exec_;      // name of the executable currently running in RUN_MISSION

    // Base-zone stability tracking
    double       base_tol_m_;            // param: radial tolerance (m) for origin proximity
    double       base_hold_s_;           // param: continuous seconds required inside base_tol_m_
    bool         base_zone_entered_;     // true while dist(x,y,origin) <= base_tol_m_
    rclcpp::Time base_zone_enter_time_;  // wall time when drone entered the base zone

    // Parameters forwarded to pouso when landing at the origin
    double pouso_xy_hold_tol_;           // param: xy_hold_tol for origin landing
    double pouso_xy_hold_stable_s_;      // param: xy_hold_stable_s for origin landing
    double pouso_xy_abort_tol_;          // param: xy_abort_tol for origin landing
    double pouso_approach_z_;            // param: approach_z for origin landing
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SupervisorTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
