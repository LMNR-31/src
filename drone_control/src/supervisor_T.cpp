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
//                         use_origin_as_base=true and the UAV is at the origin), waits
//                         for it to exit, then loops back to WAIT_TRAJ for the next
//                         cycle (infinite loop).
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
//   uav_name                (string, default "uav1")  — UAV namespace prefix
//   use_origin_as_base      (bool,   default true)    — when true (default), launch
//                             pouso with use_current_xy:=true instead of missao_P_T
//                             if the drone is within 0.1 m of (0,0) at trajectory
//                             completion; when false, always launch missao_P_T.
//   wait_after_traj_done_s  (double, default 5.0)     — seconds to wait in
//                             WAIT_BEFORE_MISSION before launching the next mission
//                             after a trajectory is considered complete.
//
// Usage:
//   ros2 run drone_control supervisor_T

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
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
      wait_after_traj_done_s_(5.0),
      wait_start_time_(0, 0, RCL_ROS_TIME),
      current_child_exec_("missao_P_T")
    {
        this->declare_parameter<std::string>("uav_name", "uav1");
        uav_name_ = this->get_parameter("uav_name").as_string();

        this->declare_parameter<bool>("use_origin_as_base", true);
        use_origin_as_base_ = this->get_parameter("use_origin_as_base").as_bool();

        this->declare_parameter<double>("wait_after_traj_done_s", 5.0);
        wait_after_traj_done_s_ = this->get_parameter("wait_after_traj_done_s").as_double();

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
            "use_origin_as_base=%s, wait_after_traj_done_s=%.1f",
            use_origin_as_base_ ? "true" : "false",
            wait_after_traj_done_s_);
    }

private:
    // ── /mavros/local_position/odom callback ──────────────────────────────
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        odom_received_ = true;
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

        // Delay elapsed — determine which executable to launch next.
        // When use_origin_as_base_=true (default) and the drone is within BASE_TOL
        // of the origin, launch pouso with use_current_xy:=true (local position).
        // Otherwise (use_origin_as_base_=false, or drone not at origin), run missao_P_T.
        static constexpr double BASE_TOL = 0.1;
        const bool at_base =
            use_origin_as_base_ &&
            odom_received_ &&
            (std::abs(current_x_) <= BASE_TOL) &&
            (std::abs(current_y_) <= BASE_TOL);

        const char * next_exec = at_base ? "pouso (use_current_xy)" : "missao_P_T";
        current_child_exec_ = next_exec;

        if (at_base) {
            RCLCPP_INFO(this->get_logger(),
                "[WAIT_BEFORE_MISSION] ⏱️  %.1f s concluídos. UAV no ponto base "
                "(x=%.2f y=%.2f) — lançando pouso com posição local…",
                wait_after_traj_done_s_, current_x_, current_y_);
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
    // Used when the drone has returned to the origin so that it lands at the
    // current local position instead of running the full missao_P_T sequence.
    pid_t fork_exec_pouso_local() {
        pid_t pid = fork();
        if (pid == 0) {
            execlp("ros2", "ros2", "run", "drone_control", "pouso",
                   "--ros-args", "-p", "use_current_xy:=true",
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
    std::string current_child_exec_;      // name of the executable currently running in RUN_MISSION
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SupervisorTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
