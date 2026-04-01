// missao_P_T.cpp — ROS 2 orchestrator node: pouso → 5 s → takeoff
//
// Executes the following sequence:
//   1. ros2 run drone_control pouso   (land the drone at current/specified XY)
//   2. Wait exactly 5 seconds
//   3. ros2 run drone_control takeoff (take off again)
//
// If `pouso` returns a non-zero exit code the node logs an error and does NOT
// proceed with takeoff (fail-safe behaviour).
//
// Usage:
//   ros2 run drone_control missao_P_T

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <atomic>

using namespace std::chrono_literals;

class MissaoPTNode : public rclcpp::Node
{
public:
  MissaoPTNode()
  : Node("missao_P_T"), stop_requested_(false)
  {
    // Fire once after 1 s, then hand off to a background thread so the
    // executor keeps spinning (log callbacks stay live).
    timer_ = this->create_wall_timer(
      1s, std::bind(&MissaoPTNode::startMissionAsync, this));

    RCLCPP_INFO(this->get_logger(),
      "🚀 missao_P_T iniciado. Sequência: pouso → 5 s → takeoff");
  }

  ~MissaoPTNode()
  {
    stop_requested_.store(true);
    if (mission_thread_.joinable()) {
      mission_thread_.join();
    }
  }

private:
  void startMissionAsync()
  {
    // Run only once
    timer_->cancel();
    mission_thread_ = std::thread(&MissaoPTNode::runMission, this);
  }

  void runMission()
  {
    // ── FASE 1: pouso ────────────────────────────────────────────────────────
    RCLCPP_INFO(this->get_logger(),
      "⬇️  [FASE 1] Iniciando pouso (ros2 run drone_control pouso)…");

    int pouso_result = std::system("ros2 run drone_control pouso");

    if (pouso_result != 0) {
      RCLCPP_ERROR(this->get_logger(),
        "❌ [FASE 1] pouso falhou com código %d. "
        "Takeoff cancelado por segurança. Encerrando.",
        pouso_result);
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "✅ [FASE 1] pouso concluído com sucesso.");

    // ── FASE 2: aguardar 5 s ─────────────────────────────────────────────────
    RCLCPP_INFO(this->get_logger(),
      "⏳ [FASE 2] Aguardando 10 segundos antes de decolar…");

    for (int i = 10; i > 0; --i) {
      if (stop_requested_.load()) {
        RCLCPP_WARN(this->get_logger(),
          "⚠️  Interrupção solicitada durante espera. Encerrando.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(),
        "  ⏱️  %d segundo(s) restante(s)…", i);
      std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO(this->get_logger(),
      "✅ [FASE 2] Espera de 5 s concluída.");

    // ── FASE 3: takeoff ──────────────────────────────────────────────────────
    RCLCPP_INFO(this->get_logger(),
      "🛫 [FASE 3] Iniciando takeoff (ros2 run drone_control takeoff)…");

    int takeoff_result = std::system("ros2 run drone_control takeoff");

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

    rclcpp::shutdown();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::thread                  mission_thread_;
  std::atomic<bool>            stop_requested_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissaoPTNode>());
  rclcpp::shutdown();
  return 0;
}
