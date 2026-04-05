// missao_P_T.cpp — ROS 2 orchestrator node: pouso → 10 s → takeoff
//
// Executes the following sequence:
//   1. ros2 run drone_control pouso   (land the drone at current/specified XY)
//   2. Wait exactly 10 seconds
//   3. ros2 run drone_control takeoff (take off again)
//
// If `pouso` returns a non-zero exit code the node logs an error and does NOT
// proceed with takeoff (fail-safe behaviour).  The process exits with a
// non-zero return code whenever any phase fails.
//
// Each sub-phase is launched as a separate subprocess via fork()/execlp() so
// that child nodes start in a clean process and never share the ROS 2
// parameter/context state of this process.
//
// Usage:
//   ros2 run drone_control missao_P_T

#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

class MissaoPTNode : public rclcpp::Node
{
public:
  MissaoPTNode()
  : Node("missao_P_T"), stop_requested_(false), exit_code_(0)
  {
    // Fire once after 1 s, then hand off to a background thread so the
    // executor keeps spinning (log callbacks stay live).
    timer_ = this->create_wall_timer(
      1s, std::bind(&MissaoPTNode::startMissionAsync, this));

    RCLCPP_INFO(this->get_logger(),
      "🚀 missao_P_T iniciado. Sequência: pouso → 10 s → takeoff");
  }

  ~MissaoPTNode()
  {
    stop_requested_.store(true);
    if (mission_thread_.joinable()) {
      mission_thread_.join();
    }
  }

  int getExitCode() const { return exit_code_.load(); }

private:
  void startMissionAsync()
  {
    // Run only once
    timer_->cancel();
    mission_thread_ = std::thread(&MissaoPTNode::runMission, this);
  }

  // Launch `ros2 run drone_control <executable>` in a child process and wait
  // for it to finish.  Returns the child's exit code, or -1 on fork failure.
  static int run_subprocess(const char * executable) {
    pid_t pid = fork();
    if (pid == 0) {
      // Child: replace image with the target executable.
      execlp("ros2", "ros2", "run", "drone_control", executable,
             static_cast<char *>(nullptr));
      // execlp only returns on failure.
      _exit(127);
    }
    if (pid < 0) {
      // fork() failed in the parent.
      return -1;
    }
    int status = 0;
    waitpid(pid, &status, 0);   // blocking wait — called from background thread
    if (WIFEXITED(status)) {
      return WEXITSTATUS(status);
    }
    return -1;   // signalled or unknown termination
  }

  void runMission()
  {
    // ── FASE 1: pouso ────────────────────────────────────────────────────────
    RCLCPP_INFO(this->get_logger(),
      "⬇️  [FASE 1] Iniciando pouso (ros2 run drone_control pouso)…");

    int pouso_result = run_subprocess("pouso");

    if (pouso_result != 0) {
      RCLCPP_ERROR(this->get_logger(),
        "❌ [FASE 1] pouso falhou com código %d. "
        "Takeoff cancelado por segurança. Encerrando.",
        pouso_result);
      exit_code_.store(1);
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "✅ [FASE 1] pouso concluído com sucesso.");

    // ── FASE 2: aguardar 10 s ────────────────────────────────────────────────
    RCLCPP_INFO(this->get_logger(),
      "⏳ [FASE 2] Aguardando 10 segundos antes de decolar…");

    for (int i = 10; i > 0; --i) {
      if (stop_requested_.load()) {
        RCLCPP_WARN(this->get_logger(),
          "⚠️  Interrupção solicitada durante espera. Encerrando.");
        exit_code_.store(1);
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(),
        "  ⏱️  %d segundo(s) restante(s)…", i);
      std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO(this->get_logger(),
      "✅ [FASE 2] Espera de 10 s concluída.");

    // ── FASE 3: takeoff ──────────────────────────────────────────────────────
    RCLCPP_INFO(this->get_logger(),
      "🛫 [FASE 3] Iniciando takeoff (ros2 run drone_control takeoff)…");

    int takeoff_result = run_subprocess("takeoff");

    if (takeoff_result != 0) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️  [FASE 3] takeoff encerrou com código %d.", takeoff_result);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "✅ [FASE 3] takeoff concluído com sucesso.");
    }

    // ── Resumo ───────────────────────────────────────────────────────────────
    RCLCPP_INFO(this->get_logger(),
      "🏁 missao_P_T concluída. pouso=%s takeoff=%s",
      pouso_result   == 0 ? "OK" : "FALHOU",
      takeoff_result == 0 ? "OK" : "AVISO");

    if (pouso_result != 0) {
      exit_code_.store(1);
    }

    rclcpp::shutdown();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::thread                  mission_thread_;
  std::atomic<bool>            stop_requested_;
  std::atomic<int>             exit_code_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissaoPTNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return node->getExitCode();
}
