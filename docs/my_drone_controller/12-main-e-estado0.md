# 12 — `main.cpp` e `fsm_state0_wait.cpp`

Este documento explica **linha a linha** os dois arquivos mais curtos do pacote:
`src/main.cpp` (ponto de entrada do executável ROS 2) e
`src/fsm_state0_wait.cpp` (Estado 0 da FSM — aguardando comando).

---

## Parte 1 — `main.cpp`

### Código completo anotado

```cpp
// ── Includes ────────────────────────────────────────────────────────────────
#include "my_drone_controller/drone_controller_completo.hpp"
// Inclui a declaração completa da classe DroneControllerCompleto e
// todas as suas dependências transitivas (ROS 2, MAVROS, msgs, etc.).

#include "rclcpp/rclcpp.hpp"
// API principal do ROS 2 em C++:
//   rclcpp::init()      — inicializa o contexto ROS 2 (DDS/rmw, parâmetros, etc.)
//   rclcpp::spin()      — entra no loop de eventos (callbacks, timers, serviços)
//   rclcpp::shutdown()  — finaliza o contexto ROS 2

// ── Função principal ─────────────────────────────────────────────────────────
int main(int argc, char ** argv)
//   argc: número de argumentos da linha de comando
//   argv: vetor de strings (argv[0] = caminho do executável, argv[1..] = args extra)
{
  // 1. Inicializa o runtime do ROS 2.
  //    Processa argumentos ROS 2 (ex.: --ros-args -p hover_altitude:=2.5),
  //    configura a camada de comunicação DDS/rmw,
  //    e prepara o sistema de logging.
  rclcpp::init(argc, argv);

  // 2. Cria uma instância compartilhada (shared_ptr) do nó principal.
  //    O construtor de DroneControllerCompleto já:
  //      - declara e lê todos os parâmetros ROS 2,
  //      - cria publishers, subscribers, services e o timer de 100 Hz.
  auto node = std::make_shared<drone_control::DroneControllerCompleto>();

  // 3. Log de boas-vindas — visível no terminal após a criação do nó.
  //    RCLCPP_INFO publica no tópico /rosout e imprime no stderr colorido.
  RCLCPP_INFO(node->get_logger(),
    "╔════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(node->get_logger(),
    "║           🚁 CONTROLADOR PRONTO PARA OPERAÇÃO              ║");
  RCLCPP_INFO(node->get_logger(),
    "║                                                            ║");
  RCLCPP_INFO(node->get_logger(),
    "║  Pressione Ctrl+C para encerrar                            ║");
  RCLCPP_INFO(node->get_logger(),
    "╚════════════════════════════════════════════════════════════╝\n");

  // 4. Entra no loop de eventos do ROS 2 (bloqueante até Ctrl+C ou shutdown).
  //    Durante o spin, o executor ROS 2 despacha:
  //      - callbacks de subscribers (waypoints, odometria, estado do FCU, etc.)
  //      - callbacks de timers     (control_loop a 100 Hz, heartbeats, etc.)
  //      - respostas de serviços   (SET_MODE, ARM/DISARM)
  //    O nó permanece ativo enquanto rclcpp::ok() retornar true.
  rclcpp::spin(node);

  // 5. Encerra o runtime do ROS 2 (liberação de recursos DDS, threads, etc.).
  //    Chamado automaticamente quando rclcpp::spin() retorna, seja por
  //    Ctrl+C (SIGINT), rclcpp::shutdown() interno, ou fim do spin_until_future.
  rclcpp::shutdown();

  // 6. Retorna 0 ao sistema operacional, indicando saída normal.
  return 0;
}
```

### Fluxo de execução

```
$ ros2 run my_drone_controller drone_node
        │
        ▼
  rclcpp::init(argc, argv)
  └─ processa --ros-args, inicializa DDS
        │
        ▼
  make_shared<DroneControllerCompleto>()
  ├─ load_parameters()    ← lê YAML / linha de comando
  ├─ setup_publishers()   ← cria todos os publishers
  ├─ setup_subscribers()  ← cria todos os subscribers
  ├─ setup_services()     ← cria service clients + timer 100 Hz
  └─ init_variables()     ← state_voo_=0, flags=false
        │
        ▼
  [log: ╔══ CONTROLADOR PRONTO ══╗]
        │
        ▼
  rclcpp::spin(node)   ← bloqueante
  ├─ a cada 10 ms: control_loop() → FSM
  ├─ ao receber /waypoints:  waypoints_callback()
  ├─ ao receber /odom:       odometry_callback()
  └─ ... (outros callbacks)
        │
        ▼  (Ctrl+C / SIGINT)
  rclcpp::shutdown()
        │
        ▼
  return 0
```

### Por que `std::make_shared` em vez de construção direta?

`rclcpp::spin(node)` espera um `std::shared_ptr<rclcpp::Node>` (ou compatível).
Além disso, o nó precisa de `shared_from_this()` internamente para criar
timers e callbacks — o que só funciona com `shared_ptr`.

---

## Parte 2 — `fsm_state0_wait.cpp`

### Código completo anotado

```cpp
// ── Include do cabeçalho do controlador ──────────────────────────────────────
#include "my_drone_controller/drone_controller_completo.hpp"
// Traz a declaração de DroneControllerCompleto, necessária para implementar
// seus métodos fora do arquivo de cabeçalho.

namespace drone_control {
// Mesmo namespace da classe — obrigatório para que a definição seja reconhecida.

// ============================================================
// FSM STATE 0 — AGUARDANDO WAYPOINT
// ============================================================

void DroneControllerCompleto::handle_state0_wait_waypoint()
// Implementação do método handle_state0_wait_waypoint().
// Chamado em cada ciclo de 10 ms pelo control_loop() quando state_voo_ == 0.
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),   // logger do nó atual
    *this->get_clock(),   // clock do nó (necessário para throttle)
    10000,                // throttle: máximo 1 mensagem a cada 10 000 ms (= 10 s)
    "⏳ Aguardando novo comando de waypoint para decolar...");
  // → Sem esse throttle, o terminal seria inundado com 100 mensagens/segundo.
  // → A mensagem só reaparece a cada 10 s, lembrando o operador que o sistema
  //   está ocioso mas funcional.
}

}  // namespace drone_control
```

### Por que o Estado 0 faz tão pouco?

O Estado 0 é o estado **de repouso** da FSM. O drone está no chão, desarmado, esperando um comando externo. Não há nada para calcular nem publicar — qualquer setpoint publicado nesse momento seria ignorado pelo FCU (que está desarmado).

A única ação útil é o log para informar que o controlador está vivo e aguardando. O throttle de 10 s evita spam sem perder a visibilidade do estado.

### Como o Estado 0 termina?

O Estado 0 termina quando `waypoints_callback()` ou `waypoints_4d_callback()` recebem um comando de takeoff (1 waypoint, Z ≥ threshold). Internamente, essas callbacks chamam `handle_single_takeoff_waypoint_command()` que executa:

```cpp
state_voo_ = 1;  // transição 0 → 1 (DECOLAGEM)
```

A partir do próximo ciclo do `control_loop()`, o despacho cai no `case 1` e `handle_state1_takeoff()` assume.

### Diagrama de transição do Estado 0

```
Estado 0 (AGUARDANDO)
  │
  │  handle_state0_wait_waypoint() a cada 10 ms
  │  (log throttled a cada 10 s)
  │
  ├─ recebe /waypoints [1 pose, z < threshold]  → trigger_landing() → state_voo_=4
  ├─ recebe /waypoints [1 pose, z ≥ threshold]  → state_voo_ = 1 (TAKEOFF)
  ├─ recebe /waypoints [2+ poses]               → armazena; permanece em 0
  ├─ recebe /waypoint_goal_4d [1 ponto]         → state_voo_ = 1 (TAKEOFF 4D)
  └─ recebe /waypoints_4d [2+ pontos]           → armazena; permanece em 0
```

> **Nota:** Quando 2+ waypoints chegam no Estado 0, eles são armazenados em `trajectory_waypoints_` mas `state_voo_` permanece em 0. O drone só decola quando recebe um waypoint de takeoff (1 pose, Z ≥ threshold) *depois* disso.

---

## Resumo comparativo

| Aspecto | `main.cpp` | `fsm_state0_wait.cpp` |
|---|---|---|
| Papel | Ponto de entrada do executável | Estado ocioso da FSM |
| Linhas de código | 18 | 15 |
| Frequência de execução | 1×, na inicialização | 100 Hz (throttle a 10 s) |
| Publica setpoints? | Não | Não |
| Altera `state_voo_`? | Não | Não |
| Ação principal | `rclcpp::spin()` | `RCLCPP_INFO_THROTTLE` |
