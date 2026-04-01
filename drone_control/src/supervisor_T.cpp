// supervisor_T.cpp — ROS 2 node that launches missao_P_T each time a
// trajectory finishes, and resets automatically so it fires again for the
// next trajectory.
//
// Detection logic (mirrors supervisor.cpp):
//   • /trajectory_finished == true    → trajectory done
//   • /trajectory_progress >= 100     → trajectory done
//   • /trajectory_finished == false   → new trajectory started, reset guards
//   • /trajectory_progress  <  99.9   → new trajectory started, reset guards
//     (99.9 threshold avoids spurious resets from float jitter near 100.0)
//
// Launch pattern: fork() + execlp("ros2", ...) identical to supervisor.cpp.
// Only one instance of missao_P_T runs at a time; extras are queued.
// Child processes are reaped via WNOHANG to prevent zombies.
//
// Usage:
//   ros2 run drone_control supervisor_T

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <memory>
#include <queue>

class SupervisorTNode : public rclcpp::Node {
public:
    SupervisorTNode()
    : Node("supervisor_T"),
      missao_pid_(-1),
      trajectory_done_received_(false),
      trajectory_active_(false)
    {
        progress_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/trajectory_progress", 10,
            std::bind(&SupervisorTNode::progress_callback, this, std::placeholders::_1));

        finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/trajectory_finished", 10,
            std::bind(&SupervisorTNode::finished_callback, this, std::placeholders::_1));

        // Periodic timer: reap finished missao_P_T and drain the launch queue.
        queue_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SupervisorTNode::process_queue, this));

        RCLCPP_INFO(this->get_logger(),
            "supervisor_T iniciado — monitorando /trajectory_progress e "
            "/trajectory_finished; missao_P_T será disparado a cada término de trajetória.");
    }

private:
    // ── Helpers ────────────────────────────────────────────────────────────
    void on_trajectory_started() {
        if (!trajectory_active_) {
            trajectory_active_       = true;
            trajectory_done_received_ = false;
            RCLCPP_INFO(this->get_logger(),
                "▶️  Nova trajetória detectada — guardas resetados, aguardando conclusão.");
        }
    }

    void on_trajectory_finished() {
        if (!trajectory_done_received_) {
            trajectory_done_received_ = true;
            pending_queue_.push(true);   // token indicating a pending launch
            RCLCPP_INFO(this->get_logger(),
                "🏁 Trajetória concluída — missao_P_T enfileirado (fila=%zu).",
                pending_queue_.size());
        }
    }

    // ── /trajectory_progress callback ─────────────────────────────────────
    // NOTE: all callbacks and the queue timer share the same single-threaded
    // executor, so there is no concurrent access to the state variables.
    void progress_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data < 99.9f) {
            // Progress < 99.9 → a trajectory is running (or a new one just started).
            // Using 99.9 instead of 100.0 to avoid spurious resets from floating-point
            // values that hover just below the completion threshold.
            on_trajectory_started();
        } else {
            // Progress >= 100 → trajectory finished.
            RCLCPP_INFO(this->get_logger(),
                "📊 Progresso %.1f%% recebido — trajetória concluída.", msg->data);
            on_trajectory_finished();
        }
    }

    // ── /trajectory_finished callback ──────────────────────────────────────
    void finished_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            // False → new trajectory starting; reset so we can fire again.
            on_trajectory_started();
        } else {
            // True → trajectory finished.
            RCLCPP_INFO(this->get_logger(),
                "📨 /trajectory_finished=true recebido — trajetória concluída.");
            on_trajectory_finished();
        }
    }

    // ── Periodic: reap finished process, launch next from queue ───────────
    void process_queue() {
        // Reap the previous missao_P_T if it has finished.
        if (missao_pid_ > 0) {
            int status = 0;
            pid_t result = waitpid(missao_pid_, &status, WNOHANG);
            if (result == missao_pid_) {
                if (WIFEXITED(status)) {
                    RCLCPP_INFO(this->get_logger(),
                        "✅ missao_P_T (PID %d) finalizado com código %d.",
                        static_cast<int>(missao_pid_), WEXITSTATUS(status));
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "⚠️  missao_P_T (PID %d) encerrou de forma anormal.",
                        static_cast<int>(missao_pid_));
                }
                missao_pid_ = -1;
            } else if (result < 0) {
                // Process no longer exists (already reaped or invalid).
                missao_pid_ = -1;
            } else {
                // Still running; do not launch another instance yet.
                return;
            }
        }

        // Nothing queued — nothing to do.
        if (pending_queue_.empty()) {
            return;
        }

        pending_queue_.pop();

        RCLCPP_INFO(this->get_logger(),
            "🚀 [SUPERVISOR_T] Lançando missao_P_T…");

        pid_t pid = fork();
        if (pid == 0) {
            // Child: exec missao_P_T.
            execlp("ros2", "ros2", "run", "drone_control", "missao_P_T",
                   static_cast<char*>(nullptr));
            // exec failed
            _exit(1);
        } else if (pid > 0) {
            missao_pid_ = pid;
            RCLCPP_INFO(this->get_logger(),
                "missao_P_T iniciado (PID %d).", static_cast<int>(pid));
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "fork() falhou ao iniciar missao_P_T!");
        }

        // After launching, mark trajectory as inactive so the next
        // trajectory-started signal properly resets guards.
        trajectory_active_ = false;
    }

    // ── Members ────────────────────────────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr progress_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    finished_sub_;
    rclcpp::TimerBase::SharedPtr                            queue_timer_;

    pid_t              missao_pid_;            // PID of running missao_P_T (-1 if none)
    bool               trajectory_done_received_;  // guard: fire once per trajectory
    bool               trajectory_active_;         // true while a trajectory is in progress
    std::queue<bool>   pending_queue_;             // pending launches (token queue)
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SupervisorTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
