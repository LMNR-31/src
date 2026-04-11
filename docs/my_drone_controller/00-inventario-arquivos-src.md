# Inventário Completo de Arquivos — `my_drone_controller/src`

> **Objetivo deste documento:** listar e descrever **todos** os arquivos presentes em
> `my_drone_controller/src`, indicando para cada um o papel/responsabilidade, as
> principais funções/classes que contém, a relação com a FSM e/ou com o nó
> `DroneControllerCompleto`, e **o que cada bloco de código faz**.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização com o diretório: 2026-04-11.

---

## Índice rápido

| # | Arquivo | Categoria | Doc detalhada |
|---|---------|-----------|---------------|
| 1 | [`main.cpp`](#1-maincpp) | Ponto de entrada | [12-main-e-estado0.md](12-main-e-estado0.md) |
| 2 | [`drone_controller_completo.cpp`](#2-drone_controller_completocpp) | Nó ROS 2 principal | [03](03-drone_controller_completo-parte1-setup.md) · [04](04-drone_controller_completo-parte2-loop-e-setpoints.md) · [11](11-subscribers-e-callbacks.md) |
| 3 | [`fsm_state0_wait.cpp`](#3-fsm_state0_waitcpp) | FSM — Estado 0 | [12-main-e-estado0.md](12-main-e-estado0.md) |
| 4 | [`fsm_takeoff.cpp`](#4-fsm_takeoffcpp) | FSM — Estado 1 | [05-fsm_takeoff.md](05-fsm_takeoff.md) |
| 5 | [`fsm_hover.cpp`](#5-fsm_hovercpp) | FSM — Estado 2 | [06-fsm_hover.md](06-fsm_hover.md) |
| 6 | [`fsm_trajectory.cpp`](#6-fsm_trajectorycpp) | FSM — Estado 3 | [07-fsm_trajectory.md](07-fsm_trajectory.md) |
| 7 | [`fsm_landing.cpp`](#7-fsm_landingcpp) | FSM — Estado 4 | [08-fsm_landing.md](08-fsm_landing.md) |
| 8 | [`command_queue.cpp`](#8-command_queuecpp) | Fila de auditoria | [14-command-queue.md](14-command-queue.md) |
| 9 | [`waypoint_validation.cpp`](#9-waypoint_validationcpp) | Validação de entrada | [13-waypoint-validation.md](13-waypoint-validation.md) |
| 10 | [`Drone_codegen.cpp`](#10-drone_codegencpp) | Controlador PID codegen | [15-codegen-planner-e-controlador.md](15-codegen-planner-e-controlador.md) |
| 11 | [`TrajectoryPlanner_codegen.cpp`](#11-trajectoryplanner_codegencpp) | Planner polinomial codegen | [15-codegen-planner-e-controlador.md](15-codegen-planner-e-controlador.md) |
| 12 | [`cos.cpp`](#12-coscpp) | Helper matemático codegen | [16-codegen-helpers-matematicos.md](16-codegen-helpers-matematicos.md#5-coscpp--stub-vazio-compatibilidade-de-codegen) |
| 13 | [`minOrMax.cpp`](#13-minormaxcpp) | Helper matemático codegen | [16-codegen-helpers-matematicos.md](16-codegen-helpers-matematicos.md#3-minormaxcpp--saturação-de-vetores-e-escalares) |
| 14 | [`mldivide.cpp`](#14-mldividecpp) | Helper matemático codegen | [16-codegen-helpers-matematicos.md](16-codegen-helpers-matematicos.md#4-mldividecpp--solver-de-sistemas-lineares-por-eliminação-gaussiana-com-pivotamento-parcial) |
| 15 | [`rt_nonfinite.cpp`](#15-rt_nonfinitecpp) | Helper matemático codegen | [16-codegen-helpers-matematicos.md](16-codegen-helpers-matematicos.md#2-rt_nonfinitecpp--constantes-globais-nan--inf) |
| 16 | [`sqrt.cpp`](#16-sqrtcpp) | Helper matemático codegen | [16-codegen-helpers-matematicos.md](16-codegen-helpers-matematicos.md#6-sqrtcpp--stub-vazio-compatibilidade-de-codegen) |

---

## 1. `main.cpp`

### Papel / Responsabilidade

Ponto de entrada do executável `drone_node`. Inicializa o *runtime* ROS 2, instancia
o nó `DroneControllerCompleto` e entrega o controle ao *spin loop* do ROS 2. Sem
lógica de negócio — toda a inteligência está no nó instanciado.

### Principais funções / classes

| Símbolo | Descrição |
|---------|-----------|
| `main(int argc, char ** argv)` | Única função; inicializa ROS 2, cria o nó e faz spin. |

### Relação com a FSM / `DroneControllerCompleto`

É o criador do objeto `DroneControllerCompleto`. O construtor do nó configura
publishers, subscribers, serviços e timers; depois `rclcpp::spin()` mantém o
*event loop* ativo para que os callbacks e o timer `control_loop()` funcionem.

### O que cada bloco de código faz

```cpp
// Bloco 1 — Includes
#include "my_drone_controller/drone_controller_completo.hpp"
#include "rclcpp/rclcpp.hpp"
```
> Inclui a declaração completa do nó (`drone_controller_completo.hpp`) e o cabeçalho
> ROS 2 necessário para `rclcpp::init`, `rclcpp::spin` e `rclcpp::shutdown`.

```cpp
// Bloco 2 — Inicialização do runtime ROS 2
rclcpp::init(argc, argv);
```
> Inicializa o middleware RCL/DDS a partir dos argumentos da linha de comando (permite
> remapeamento de tópicos, parâmetros externos, etc.). Deve ser chamado antes de
> qualquer operação ROS 2.

```cpp
// Bloco 3 — Instanciação do nó
auto node = std::make_shared<drone_control::DroneControllerCompleto>();
```
> Cria o único nó do processo usando ponteiro gerenciado. O construtor de
> `DroneControllerCompleto` carrega parâmetros, cria publishers/subscribers/serviços
> e inicia o timer de controle (100 Hz).

```cpp
// Bloco 4 — Banner de boas-vindas
RCLCPP_INFO(node->get_logger(), "╔═══ … ╗");
```
> Imprime no terminal um banner visual confirmando que o controlador foi inicializado
> com sucesso. Útil para depuração em sessões tmux/screen.

```cpp
// Bloco 5 — Spin e encerramento
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
```
> `spin()` bloqueia o *thread* principal e despacha callbacks ROS 2 (mensagens
> recebidas, timers, etc.) até que `Ctrl+C` ou `rclcpp::shutdown()` sejam chamados.
> `shutdown()` libera todos os recursos DDS e encerra o processo de forma limpa.

---

## 2. `drone_controller_completo.cpp`

### Papel / Responsabilidade

Implementação principal do nó ROS 2 `DroneControllerCompleto`. Concentra:
setup de publishers/subscribers/serviços, carregamento de parâmetros, loop de controle
a 100 Hz (`control_loop()`), publicação de setpoints MAVROS, callbacks de entrada
(waypoints, odometria, estado FCU, yaw override) e a máquina de estados de alto nível
que delega para os arquivos FSM especializados.

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `DroneControllerCompleto()` (construtor) | Carrega parâmetros, configura pub/sub/svc/timer. |
| `load_parameters()` | Lê parâmetros ROS 2 para `config_` (altitudes, timeouts, tópicos). |
| `setup_publishers()` | Cria todos os publishers do nó. |
| `setup_subscribers()` | Cria todos os subscribers e define callbacks. |
| `setup_services()` | Cria clientes de serviço MAVROS (arm, set_mode). |
| `init_variables()` | Zera/inicializa variáveis de estado internas. |
| `control_loop()` | Timer callback 100 Hz — aciona a FSM e o watchdog de setpoint. |
| `publishPositionTarget(…)` | Publica setpoint de posição (sem yaw). |
| `publishPositionTargetWithYaw(…)` | Publica setpoint de posição com yaw fixo. |
| `publishPositionTargetWithVelocityAndYaw(…)` | Publica posição + velocidade feedforward + yaw. |
| `publish_hold_setpoint()` | Republica o último setpoint (watchdog anti-timeout). |
| `waypoints_callback()` / `waypoints_4d_callback()` | Recebem listas de waypoints e roteiam (pouso, decolagem, trajetória). |
| `waypoint_goal_callback()` / `waypoint_goal_4d_callback()` | Recebem waypoint único de hover/decolagem. |
| `state_callback()` | Atualiza `current_state_` (armed, mode) vindo do MAVROS. |
| `extended_state_callback()` | Atualiza `last_extended_state_` (landed_state). |
| `odometry_callback()` | Atualiza posição e velocidade NED para o controlador. |
| `yaw_override_callback()` | Ativa sobrescrição de yaw externo. |
| `sanitize_takeoff_xy()` | Substitui XY próximo de (0,0) pela última latch pose. |
| `trigger_landing()` | Enfileira LAND e transiciona para estado 4. |
| `autopilot_indicates_landing()` | Verifica `landed_state` do FCU. |
| `compute_yaw_for_trajectory_waypoint()` | Calcula yaw de "apontar para o próximo WP". |

### Relação com a FSM / `DroneControllerCompleto`

É o nó principal. O método `control_loop()` implementa o *switch* da FSM chamando:
`handle_state0_wait_waypoint()`, `handle_state1_takeoff()`, `handle_state2_hover()`,
`handle_state3_trajectory()` e `handle_state4_landing()` — cada um definido nos
arquivos `fsm_*.cpp` correspondentes.

### O que cada bloco de código faz

> Para detalhamento completo linha-a-linha consulte
> [03 — Setup (Parte 1)](03-drone_controller_completo-parte1-setup.md) e
> [04 — Loop e Setpoints (Parte 2)](04-drone_controller_completo-parte2-loop-e-setpoints.md).
> Abaixo um resumo orientado a blocos:

**Bloco: Includes e namespace**
```cpp
#include "my_drone_controller/drone_controller_completo.hpp"
namespace drone_control { … }
```
> Inclui a declaração completa da classe (membros, tipos, constantes) e delimita
> todas as definições no namespace `drone_control` para evitar colisões de nomes.

**Bloco: Construtor — sequência de inicialização**
```cpp
DroneControllerCompleto::DroneControllerCompleto()
: Node("drone_controller_completo")
{
  load_parameters();
  setup_publishers();
  setup_subscribers();
  setup_services();
  init_variables();
  control_timer_ = create_wall_timer(…, std::bind(&…::control_loop, this));
}
```
> 1. Registra o nó com o nome `"drone_controller_completo"` no grafo ROS 2.
> 2. Carrega todos os parâmetros ROS 2 para `config_`.
> 3. Cria publishers, subscribers e clientes de serviço.
> 4. Inicializa todas as variáveis de estado.
> 5. Cria o timer de 10 ms que invoca `control_loop()` a 100 Hz.

**Bloco: `load_parameters()`**
```cpp
config_.hover_altitude = this->get_parameter("hover_altitude").as_double();
// … outros parâmetros …
```
> Lê cada parâmetro ROS 2 declarado (e.g., `hover_altitude`, `min_altitude`,
> `landing_timeout`, nomes de tópicos) e preenche a struct `DroneConfig config_`.
> Valores padrão são definidos em `drone_config.h` caso o parâmetro não seja
> fornecido externamente (via YAML ou linha de comando).

**Bloco: `setup_publishers()`**
```cpp
setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
  "/uav1/mavros/setpoint_raw/local", rclcpp::QoS(1));
state_voo_pub_ = create_publisher<std_msgs::msg::Int32>(
  "/drone_controller/state_voo",
  rclcpp::QoS(1).transient_local());
```
> Cria todos os canais de saída do nó:
> setpoints para o FCU, estado da FSM (QoS `transient_local` para que novos
> assinantes recebam o último valor imediatamente), progresso da trajetória,
> waypoint atingido, latch pose e `trajectory_finished`.

**Bloco: `setup_subscribers()`**
```cpp
waypoints_sub_ = create_subscription<…>("/waypoints", …,
  std::bind(&DroneControllerCompleto::waypoints_callback, this, _1));
```
> Registra todos os assinantes de entrada (waypoints, odometria, estado FCU, yaw
> override) com QoS adequado a cada fonte. Guards `skip_self_*` evitam que o nó
> reaja às próprias publicações de status.

**Bloco: `control_loop()` — dispatcher da FSM**
```cpp
void DroneControllerCompleto::control_loop() {
  watchdog_setpoint();          // garante ≥ 1 setpoint a cada 20 Hz
  publish_state_voo_heartbeat();

  switch (state_voo_) {
    case 0: handle_state0_wait_waypoint(); break;
    case 1: handle_state1_takeoff();       break;
    case 2: handle_state2_hover();         break;
    case 3: handle_state3_trajectory();    break;
    case 4: handle_state4_landing();       break;
  }
}
```
> Chamado a cada 10 ms. O `watchdog_setpoint()` republica o último setpoint se
> nenhum foi publicado nas últimas 50 ms, garantindo que o FCU não perca o heartbeat
> OFFBOARD. Em seguida o `switch` direciona para a função FSM correta.

**Bloco: `publishPositionTarget*(…)`**
```cpp
mavros_msgs::msg::PositionTarget msg;
msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
msg.type_mask = MASK_POS_ONLY;   // ignora vel/accel/yaw
// … preenche position e publica …
setpoint_pub_->publish(msg);
```
> Monta e publica a mensagem `PositionTarget` no tópico MAVROS. A máscara
> (`type_mask`) controla quais campos o FCU deve usar: somente posição, posição
> + yaw, ou posição + velocidade feedforward + yaw.

---

## 3. `fsm_state0_wait.cpp`

### Papel / Responsabilidade

Implementa o **Estado 0** (ocioso) da FSM. O drone está no chão ou com o
controlador desarmado, aguardando um novo comando de waypoint via tópico.
É o estado padrão após inicialização e após o ciclo completo pouso → reset.

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `handle_state0_wait_waypoint()` | Emite log throttled a cada 10 s; sem lógica de controle. |

### Relação com a FSM / `DroneControllerCompleto`

Chamado por `control_loop()` quando `state_voo_ == 0`. A transição para o Estado 1
ocorre nas callbacks `waypoints_callback()` / `waypoint_goal_callback()` quando um
comando válido de decolagem é recebido (lá `state_voo_` é setado para `1`).

### O que cada bloco de código faz

```cpp
// Bloco único — log periódico
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
  "⏳ Aguardando novo comando de waypoint para decolar...");
```
> Imprime uma mensagem de status no máximo uma vez a cada 10 000 ms (10 s) para
> não inundar o terminal. A função não publica setpoints nem interage com o FCU:
> o drone permanece no chão com o motor desarmado (DISARM).

---

## 4. `fsm_takeoff.cpp`

### Papel / Responsabilidade

Implementa o **Estado 1** (decolagem) da FSM. Contém a sequência completa de
pré-armar o FCU conforme boas práticas PX4/MAVROS: *streaming* de setpoints antes
do ARM, separação da solicitação de OFFBOARD e ARM, e confirmação por *polling*
de `current_state_`.

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `handle_state1_takeoff()` | Orquestra os 5 passos da decolagem. |
| `stream_initial_setpoints()` | Publica setpoints antes de solicitar OFFBOARD (pré-ARM stream). |
| `request_arm_and_offboard_activation()` | Solicita **somente** o modo OFFBOARD (ARM é separado). |
| `wait_for_offboard_mode()` | Aguarda FCU confirmar `mode == "OFFBOARD"`. |
| `stream_post_offboard_setpoints()` | Continua streaming por 1,5 s após OFFBOARD confirmado. |
| `wait_for_offboard_arm_confirmation()` | Aguarda `armed == true && mode == "OFFBOARD"`. |
| `publish_takeoff_climb_setpoint(alt)` | Publica setpoint de subida com alvo fixo. |
| `finalize_takeoff_on_altitude_reached(alt)` | Verifica chegada à altitude e transiciona para Estado 2. |

### Relação com a FSM / `DroneControllerCompleto`

Chamado por `control_loop()` quando `state_voo_ == 1`. Ao atingir a altitude alvo
(`takeoff_target_z_`), seta `state_voo_ = 2` (hover) e enfileira o comando HOVER
em `cmd_queue_`.

### O que cada bloco de código faz

**Bloco: `handle_state1_takeoff()` — orquestrador em 5 passos**
```cpp
if (!initial_stream_done_)     { stream_initial_setpoints();             return; }
if (!offboard_activated_)      { request_arm_and_offboard_activation();  return; }
if (!offboard_mode_confirmed_) { wait_for_offboard_mode();               return; }
if (!post_offboard_stream_done_){ stream_post_offboard_setpoints();      return; }
if (!arm_requested_)           { request_arm(); arm_requested_ = true;   return; }
if (!activation_confirmed_)    { if (!wait_for_offboard_arm_confirmation()) return; }
publish_takeoff_climb_setpoint(takeoff_target_z_);
finalize_takeoff_on_altitude_reached(takeoff_target_z_);
```
> Cada etapa é protegida por uma guarda booleana; ao falhar, retorna imediatamente
> para ser reexecutada no próximo ciclo de 10 ms. Isso cria um pipeline sequencial
> não bloqueante dentro do *timer callback*.

**Bloco: `stream_initial_setpoints()` — pré-ARM streaming**
```cpp
publishPositionTarget(last_waypoint_goal_.pose.position.x, …, MASK_POS_ONLY);
initial_stream_count_++;
if (initial_stream_count_ >= INITIAL_STREAM_THRESHOLD)
  initial_stream_done_ = true;
```
> Publica setpoints na posição alvo de decolagem por `INITIAL_STREAM_THRESHOLD`
> ciclos (~200 ms a 100 Hz). O FCU PX4 exige este pré-aquecimento para aceitar
> a transição para OFFBOARD sem rejeitar o pedido de ARM.

**Bloco: `request_arm_and_offboard_activation()` — solicitação de OFFBOARD isolada**
```cpp
request_offboard();
offboard_activated_ = true;
activation_time_ = this->now();
```
> Envia a solicitação de modo OFFBOARD via serviço MAVROS. **ARM não é enviado
> aqui** — enviá-los simultaneamente faz o FCU rejeitar o ARM porque a transição
> de modo ainda não completou. O timestamp `activation_time_` é salvo para
> detectar timeout de confirmação.

**Bloco: `wait_for_offboard_mode()` — polling de confirmação de OFFBOARD**
```cpp
if (current_state_.mode == "OFFBOARD") {
  offboard_mode_confirmed_ = true;
  activation_time_ = this->now();  // reinicia relógio para o timeout do ARM
  return;
}
if ((this->now() - activation_time_).seconds() > config_.offboard_confirm_timeout) {
  // reset de todos os flags → recomeça do passo 1
}
```
> Cada ciclo de 10 ms verifica se o FCU reportou `mode == "OFFBOARD"`. Em caso de
> timeout, **todos os flags são resetados** para que a sequência recomece do zero
> (re-streaming + re-solicitação de OFFBOARD).

**Bloco: `stream_post_offboard_setpoints()` — streaming de 1,5 s pós-OFFBOARD**
```cpp
publishPositionTarget(…);
post_offboard_stream_count_++;
if (post_offboard_stream_count_ >= POST_OFFBOARD_STREAM_THRESHOLD)
  post_offboard_stream_done_ = true;
```
> Guia PX4/MAVROS: mesmo após aceitar OFFBOARD, o FCU pode recusar ARM se o
> *stream* não estiver estável. Publicar 150 setpoints × 10 ms = 1,5 s satisfaz
> o requisito.

**Bloco: `publish_takeoff_climb_setpoint(alt)` — subida**
```cpp
publishPositionTarget(x, y, target_alt, 0.0, MASK_POS_ONLY);  // 3D
// ou com yaw (4D):
publishPositionTargetWithYaw(x, y, target_alt, goal_yaw_rad_);
takeoff_counter_++;
```
> Publica a posição alvo **fixa** (calculada em `handle_single_takeoff_waypoint_command`
> e armazenada em `takeoff_target_z_`). Não recalcula com base em `current_z_real_`
> para evitar o *bug de ascensão infinita* (alvo que "sobe junto" com o drone).

**Bloco: `finalize_takeoff_on_altitude_reached(alt)` — transição para Hover**
```cpp
if (current_z_real_ < target_alt - config_.hover_altitude_margin) return;
cmd_queue_.confirm(*takeoff_cmd_id_, true);
state_voo_ = 2;
```
> Quando `current_z_real_` alcança `target_alt − hover_altitude_margin`, confirma
> o comando TAKEOFF na fila de auditoria, enfileira HOVER e avança a FSM para o
> Estado 2.

---

## 5. `fsm_hover.cpp`

### Papel / Responsabilidade

Implementa o **Estado 2** (hover) da FSM. O drone mantém sua posição XYZ (e opcionalmente
yaw) aguardando novos comandos. Detecta pouso iniciado pelo autopiloto e serve como
estado intermediário entre decolagem e trajetória.

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `handle_state2_hover()` | Publica setpoint de hover; detecta pouso e início de trajetória. |

### Relação com a FSM / `DroneControllerCompleto`

Chamado por `control_loop()` quando `state_voo_ == 2`.
- Transição → Estado 3: quando `controlador_ativo_ == true` (recebeu waypoints de trajetória).
- Transição → Estado 4: quando `pouso_em_andamento_ == true` ou
  `autopilot_indicates_landing()` retorna `true`.

### O que cada bloco de código faz

**Bloco: publicação do setpoint de hover**
```cpp
if (using_4d_goal_)
  publishPositionTargetWithYaw(x, y, z, goal_yaw_rad_);
else
  publishPositionTarget(x, y, z, 0.0, MASK_POS_ONLY);
```
> Mantém o drone parado na posição de `last_waypoint_goal_`. Modo 4D ativa
> controle de yaw; modo 3D ignora yaw (FCU mantém o último heading).

**Bloco: log throttled**
```cpp
RCLCPP_INFO_THROTTLE(…, 10000, "🛸 Em HOVER … Controlador: %s", …);
```
> Emite status no máximo a cada 10 s para não inundar o terminal.

**Bloco: detecção de pouso via autopiloto**
```cpp
if (autopilot_indicates_landing()) {
  trigger_landing(current_z_real_);
  return;
}
```
> `autopilot_indicates_landing()` verifica `landed_state` do FCU
> (`LANDING` ou `ON_GROUND`). Se detectado durante hover, aciona `trigger_landing()`
> que enfileira LAND e seta `state_voo_ = 4`.

**Bloco: transição para trajetória**
```cpp
if (controlador_ativo_) {
  state_voo_ = 3;
}
```
> Quando `controlador_ativo_` é `true` (setado por `waypoints_callback()` ao
> receber uma lista de waypoints de trajetória), avança para o Estado 3.

**Bloco: transição para pouso via flag direto**
```cpp
if (pouso_em_andamento_) {
  state_voo_ = 4;
}
```
> Caminho alternativo de pouso acionado pelas callbacks de waypoint (e.g.,
> waypoint de descida abaixo de `land_z_threshold`).

---

## 6. `fsm_trajectory.cpp`

### Papel / Responsabilidade

Implementa o **Estado 3** (trajetória) da FSM. Gerencia a execução de múltiplos
waypoints usando `TrajectoryPlanner_codegen` (trajetória polinomial contínua) e
`Drone_codegen` (controlador PID de posição). Detecta chegada a cada waypoint,
publica progresso, latch pose e `/trajectory_finished` ao concluir.

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `handle_state3_trajectory()` | Orquestra o Estado 3 a cada ciclo de 10 ms. |
| `detect_and_handle_landing_in_trajectory()` | Detecta pouso durante trajetória e transiciona para Estado 4. |
| `initialize_trajectory()` | Inicializa planner e controlador na primeira chamada. |
| `publish_trajectory_waypoint_setpoint(idx)` | Publica setpoint do waypoint `idx` (fallback sem planner). |
| `log_trajectory_progress(idx)` | Log throttled do progresso (% de waypoints). |
| `finalize_trajectory_complete()` | Confirma comando, publica `trajectory_finished` e volta ao Estado 2. |
| `compute_yaw_for_trajectory_waypoint(idx, at_last)` | Retorna yaw apontado para o próximo waypoint. |

### Relação com a FSM / `DroneControllerCompleto`

Chamado por `control_loop()` quando `state_voo_ == 3`.
- Usa `TrajectoryPlanner_codegen` e `Drone_codegen` (arquivos 10 e 11).
- Ao concluir todos os waypoints, publica `/trajectory_finished` e seta
  `state_voo_ = 2` (hover no último waypoint).
- Pouso durante trajetória → `state_voo_ = 4`.

### O que cada bloco de código faz

**Bloco: `detect_and_handle_landing_in_trajectory()`**
```cpp
if (!autopilot_indicates_landing()) return false;
cmd_queue_.confirm(*trajectory_cmd_id_, false);  // falha por interrupção
land_cmd_id_ = cmd_queue_.enqueue(CommandType::LAND, …);
state_voo_ = 4;
return true;
```
> Se o autopiloto sinalizar pouso, marca o comando de trajetória como **falha** na
> fila de auditoria, enfileira um comando LAND e transiciona imediatamente para
> Estado 4. Retorna `true` para que `handle_state3_trajectory()` retorne sem executar
> o resto do ciclo.

**Bloco: `initialize_trajectory()`**
```cpp
planner_.waypoints = …;  // vetor [X0..Xn, Y0..Yn, Z0..Zn]
planner_.segmentTimes = …;
planner_.numSegments = n;
planner_.init();
drone_ctrl_.init();
planner_initialized_ = true;
```
> Executado apenas uma vez (guarda `trajectory_started_`). Constrói o vetor de
> waypoints no formato esperado pelo `TrajectoryPlanner_codegen` e inicializa
> tanto o planner quanto o controlador PID (`Drone_codegen`).

**Bloco: setpoint com planner ativo**
```cpp
planner_.getNextSetpoint(elapsed, Xd, Vd, Ad);
drone_ctrl_.r = …; drone_ctrl_.dr = …;
drone_ctrl_.PositionCtrl(Xd, Vd, Ad);
publishPositionTargetWithVelocityAndYaw(Xd[0], Xd[1], Xd[2],
  Vd[0], Vd[1], drone_ctrl_.zdot_des, yaw_follow);
```
> Obtém a referência contínua do planner (posição, velocidade e aceleração desejadas).
> Alimenta a posição e velocidade atual NED no controlador `Drone_codegen`.
> Publica velocidade feedforward XY do planner **mais** `zdot_des` do PID para Z
> (correção de erro de altitude independente do planner).

**Bloco: detecção de chegada ao waypoint**
```cpp
if (dist_xy <= XY_TOL && dz <= Z_TOL && idx != last_waypoint_reached_idx_) {
  last_waypoint_reached_idx_ = idx;
  waypoint_reached_pub_->publish(…);
  mission_latch_pose_pub_->publish(…);
  current_waypoint_idx_++;
}
```
> Tolerâncias: XY = 0,10 m, Z = 0,15 m. Ao ser atingido, publica `/waypoint_reached`
> com o índice, publica a *latch pose* exata (evita fallback XY=(0,0) nas callback
> de redecolagem) e avança o índice do waypoint ativo.

**Bloco: `finalize_trajectory_complete()`**
```cpp
cmd_queue_.confirm(*trajectory_cmd_id_, true);
trajectory_finished_pub_->publish(done_msg);   // data = true
progress_publisher_->publish(100.0f);
last_waypoint_goal_.pose = trajectory_waypoints_.back();
state_voo_ = 2;
```
> Confirma o comando na fila de auditoria, publica `/trajectory_finished = true`
> (único momento em que este tópico é publicado), atualiza `last_waypoint_goal_`
> para o último waypoint da trajetória e retorna ao hover.

---

## 7. `fsm_landing.cpp`

### Papel / Responsabilidade

Implementa o **Estado 4** (pouso) da FSM. Aguarda o timeout de confirmação de
aterrissagem (`landing_timeout`), solicita DISARM ao FCU e reseta a FSM para o
Estado 0 assim que o FCU confirmar o DISARM.

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `handle_state4_landing()` | Loop de espera por confirmação de DISARM e timeout. |
| `complete_landing()` | Confirma LAND na fila, salva log, solicita DISARM. |
| `handle_state4_disarm_reset()` | Verifica se já desarmado ao receber novo waypoint; permite redecolagem imediata. |

### Relação com a FSM / `DroneControllerCompleto`

Chamado por `control_loop()` quando `state_voo_ == 4`.
- Transição → Estado 0: após `current_state_.armed == false` (DISARM confirmado pelo FCU).
- `handle_state4_disarm_reset()` é chamado pelas callbacks de waypoint para garantir que
  um comando de decolagem recebido logo após o pouso não seja descartado.

### O que cada bloco de código faz

**Bloco: `handle_state4_disarm_reset()` — redecolagem após pouso**
```cpp
if (!current_state_.armed) {
  state_voo_ = 0;
  return false;   // permite que a callback processe o novo waypoint
}
if (disarm_requested_) {
  // ainda armado, aguardando DISARM → descarta o waypoint
  return true;
}
```
> Chamado pelas callbacks de waypoint. Se o drone já desarmou (DISARM confirmado
> antes do callback chegar), avança para Estado 0 e retorna `false` para que o
> novo waypoint seja processado normalmente. Se ainda armado com DISARM pendente,
> retorna `true` para que o waypoint seja descartado.

**Bloco: `complete_landing()` — confirmação e DISARM**
```cpp
cmd_queue_.confirm(*land_cmd_id_, true);
cmd_queue_.save_log("/tmp/drone_commands.log");
disarm_requested_ = true;
pouso_em_andamento_ = false;
pouso_start_time_set_ = false;
```
> Confirma o comando LAND na fila de auditoria, persiste o histórico completo de
> comandos em `/tmp/drone_commands.log`, e seta `disarm_requested_ = true`. Os
> flags `pouso_em_andamento_` e `pouso_start_time_set_` são limpos para evitar
> que `handle_state4_landing()` entre novamente no bloco de timeout e chame
> `complete_landing()` múltiplas vezes.

**Bloco: `handle_state4_landing()` — espera por DISARM**
```cpp
if (disarm_requested_) {
  if (!current_state_.armed) {
    disarm_requested_ = false;
    state_voo_ = 0;
  }
  return;  // aguarda FCU confirmar sem executar mais nada
}
```
> Se `disarm_requested_` estiver ativo, verifica a cada 10 ms se `armed` ficou
> `false`. Ao confirmar, reseta o flag e avança para Estado 0. Enquanto aguarda,
> retorna cedo sem processar o bloco de timeout abaixo.

**Bloco: timeout de pouso**
```cpp
if (pouso_em_andamento_) {
  if (!pouso_start_time_set_) {
    pouso_start_time_ = this->now();
    pouso_start_time_set_ = true;
  }
  if ((this->now() - pouso_start_time_).seconds() > config_.landing_timeout)
    complete_landing();
}
```
> Inicia um cronômetro na primeira vez que `pouso_em_andamento_` é `true`.
> Após `config_.landing_timeout` segundos sem confirmação, chama `complete_landing()`
> para forçar o DISARM. Isso evita que o drone fique indefinidamente em Estado 4
> caso o FCU nunca reporte `landed_state`.

---

## 8. `command_queue.cpp`

### Papel / Responsabilidade

Implementa a classe `CommandQueue`: fila rastreável de comandos emitidos ao drone
com ciclo de vida completo (PENDING → CONFIRMED/FAILED/TIMEOUT). Persiste o
histórico em arquivo de log para auditoria pós-missão. Totalmente *thread-safe*
via `std::mutex`.

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `enqueue(type, data)` | Adiciona comando à fila com ID único auto-incremental. |
| `confirm(id, success)` | Marca comando como CONFIRMED ou FAILED e remove dos pendentes. |
| `check_timeouts(timeout_s)` | Varre pendentes; promove para TIMEOUT os que ultrapassaram `timeout_s`. |
| `get_history()` | Retorna cópia do histórico completo (todos os comandos). |
| `pending_count()` | Retorna número de comandos ainda PENDING. |
| `cancel_all_pending()` | Marca todos os pendentes como FAILED (usado em reset/emergência). |
| `save_log(filename)` | Serializa o histórico completo em arquivo de texto. |

### Relação com a FSM / `DroneControllerCompleto`

`cmd_queue_` é membro de `DroneControllerCompleto`. Cada transição de estado
relevante enfileira um comando (TAKEOFF, HOVER, TRAJECTORY, LAND) e o confirma
(ou falha) ao completar. O log é salvo em `/tmp/drone_commands.log` ao final do
pouso (`complete_landing()`).

### O que cada bloco de código faz

**Bloco: construtor / destrutor**
```cpp
CommandQueue::CommandQueue() : next_id_(1) {}
CommandQueue::~CommandQueue() {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.clear(); history_.clear();
}
```
> Inicializa o contador de IDs em 1. O destrutor limpa as estruturas sob lock para
> evitar *data races* se outro thread tentar acessar durante a destruição.

**Bloco: `enqueue()`**
```cpp
cmd.id = next_id_++;
cmd.status = CommandStatus::PENDING;
cmd.timestamp = std::chrono::system_clock::now();
pending_[cmd.id] = cmd;
history_.push_back(cmd);
return cmd.id;
```
> Atribui ID único, registra o timestamp de criação, insere no mapa `pending_`
> (acesso O(1) por ID) e também no vetor `history_` (preserva ordem de inserção
> para o log). Retorna o ID para que o chamador possa depois chamar `confirm()`.

**Bloco: `confirm()`**
```cpp
it->second.status = success ? CONFIRMED : FAILED;
it->second.confirm_time = now;
// atualiza mesma entrada em history_
pending_.erase(it);
return true;
```
> Remove o comando dos pendentes e atualiza o status tanto em `pending_` quanto em
> `history_` sob o mesmo lock, garantindo consistência para leitores que chamem
> `get_history()` concorrentemente.

**Bloco: `check_timeouts()`**
```cpp
for (auto it = pending_.begin(); it != pending_.end(); ) {
  if (elapsed > timeout_seconds) {
    it->second.status = TIMEOUT;
    // espelha em history_
    timed_out.push_back(it->second.id);
    it = pending_.erase(it);
  } else { ++it; }
}
return timed_out;
```
> Varre todos os pendentes e promove para TIMEOUT os que ultrapassaram o limite.
> Atualiza o histórico sob o mesmo lock para evitar *data race* entre
> `check_timeouts()` e `save_log()`. Retorna lista de IDs expirados para o chamador
> logar avisos.

**Bloco: `cancel_all_pending()`**
```cpp
for (auto & [id, cmd] : pending_) {
  cmd.status = FAILED;
  // espelha em history_
}
pending_.clear();
```
> Útil em cenários de emergência (e.g., reset forçado). Marca todos os pendentes
> como FAILED e esvazia o mapa, sem perder o histórico.

**Bloco: `save_log()`**
```cpp
file << "[timestamp] ID=... | TIPO=... | STATUS=... | DADOS={...} | TEMPO=...s\n";
```
> Serializa o histórico em formato texto legível, incluindo timestamps locais,
> dados arbitrários (`std::map<string,string>`) e tempo decorrido até confirmação
> ou falha. Usa `localtime_r` (Unix) / `localtime_s` (Windows) para portabilidade.

---

## 9. `waypoint_validation.cpp`

### Papel / Responsabilidade

Funções puras de validação de waypoints recebidos. Verifica NaN/Inf, limites de
altitude (mínimo/máximo configuráveis) e distância XY máxima. Intenção de pouso
(Z abaixo de `land_z_threshold`) bypass a verificação de altitude mínima.

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `validate_waypoint(msg, config)` | Valida `PoseStamped`; retorna `false` em caso de falha. |
| `validate_pose(pose, config)` | Wrapper que converte `Pose` para `PoseStamped` e delega. |

### Relação com a FSM / `DroneControllerCompleto`

Chamada nas callbacks de entrada (`waypoints_callback`, `waypoints_4d_callback`,
`waypoint_goal_callback`) **antes** de qualquer mudança de estado da FSM. Um
waypoint inválido é descartado com log de aviso — o estado da FSM não muda.

### O que cada bloco de código faz

**Bloco: verificação de NaN/Inf**
```cpp
if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z)) return false;
if (std::isinf(pos.x) || std::isinf(pos.y) || std::isinf(pos.z)) return false;
```
> Primeira barreira: rejeita imediatamente qualquer coordenada inválida (produzida
> por sensores com falha ou deserialização corrompida).

**Bloco: bypass de altitude para intenção de pouso**
```cpp
if (pos.z >= config.land_z_threshold) {
  // checa min_altitude e max_altitude
}
```
> Se `pos.z < land_z_threshold`, o waypoint é interpretado como comando de pouso
> e as verificações de altitude mínima/máxima são puladas. Isso permite enviar
> Z = 0,0 m sem rejeição mesmo que `min_altitude` seja 1,0 m.

**Bloco: limites de altitude**
```cpp
if (pos.z < config.min_altitude) { RCLCPP_WARN(…); return false; }
if (pos.z > config.max_altitude) { RCLCPP_WARN(…); return false; }
```
> Rejeita waypoints fora do envelope de voo configurado, emitindo mensagem de aviso
> com os valores rejeitados e os limites vigentes para facilitar o diagnóstico.

**Bloco: limite de distância XY**
```cpp
if (std::abs(pos.x) > config.max_waypoint_distance) return false;
if (std::abs(pos.y) > config.max_waypoint_distance) return false;
```
> Impede que coordenadas XY absurdamente grandes (erros de frame ou de unidade)
> causem movimentos perigosos. A verificação é independente para X e Y.

---

## 10. `Drone_codegen.cpp`

### Papel / Responsabilidade

Controlador de posição PID gerado a partir de código MATLAB/Simulink. Recebe
posição e velocidade atuais do drone (NED) e a referência do planner (Xd, Vd, Ad)
e calcula ângulos de atitude desejados (roll/pitch) e velocidade vertical (`zdot_des`).

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `Drone_codegen::init()` | Zera estados internos e inicializa ganhos PID. |
| `Drone_codegen::PositionCtrl(Xd, Vd, Ad)` | Executa um passo do controlador PID de posição. |

### Relação com a FSM / `DroneControllerCompleto`

Usado exclusivamente no Estado 3 (`fsm_trajectory.cpp`), chamado a cada ciclo de
10 ms quando `planner_initialized_ == true`. A saída `zdot_des` é usada diretamente
como componente Z da mensagem `PositionTarget` enviada ao MAVROS.

### O que cada bloco de código faz

**Bloco: `init()` — inicialização de ganhos e estado**
```cpp
obj->g = 9.81; obj->dt = 0.01; obj->m = 1.25;
for (int i = 0; i < 3; i++) {
  obj->r[i] = obj->dr[i] = obj->r_err_sum[i] = 0.0;
}
obj->kP_pos[0] = 4.2; obj->kI_pos[0] = 0.93; obj->kD_pos[0] = 3.0;
// … demais ganhos …
```
> Zera todos os integradores (erros acumulados) e define os ganhos PID para X, Y
> e Z. A massa `m = 1,25 kg` e o passo `dt = 0,01 s` (100 Hz) correspondem ao
> modelo do drone configurado no MATLAB.

**Bloco: `PositionCtrl(Xd, Vd, Ad)` — cálculo do erro e PID**
```cpp
obj->r_err[i] = obj->r_des[i] - obj->r[i];           // erro posição
double vel_error = obj->dr_des[i] - obj->dr[i];        // erro velocidade
obj->r_err_sum[i] += obj->r_err[i] * p_cmd;           // integrador
dr_err[i] = kP * r_err + kI * r_err_sum + kD * vel_err + Ad[i]; // PD + feedforward
```
> Calcula o erro de posição e velocidade para X, Y, Z. Acumula o integrador I.
> Soma o termo feedforward de aceleração `Ad[i]` para antecipar a referência
> polinomial do planner.

**Bloco: mapeamento para atitude**
```cpp
obj->theta_des = -dr_err[0] / obj->g;  // pitch (X)
obj->phi_des   =  dr_err[1] / obj->g;  // roll  (Y)
obj->zdot_des  = Vd[2] + kP_pos[2] * obj->r_err[2]; // Z-dot
```
> Converte aceleração desejada em ângulo de atitude via modelo cinemático inverso
> simplificado (ângulos pequenos). `zdot_des` é a velocidade vertical desejada
> passada ao PX4 via setpoint.

**Bloco: saturação de segurança**
```cpp
obj->phi_des   = std::max(std::min(obj->phi_des,   0.785), -0.785);
obj->theta_des = std::max(std::min(obj->theta_des, 0.785), -0.785);
```
> Limita os ângulos de atitude em ±45° (0,785 rad) para evitar manobras
> perigosas em caso de erro grande no controlador.

---

## 11. `TrajectoryPlanner_codegen.cpp`

### Papel / Responsabilidade

Planner de trajetória polinomial gerado a partir de código MATLAB/Simulink. Interpola
posição, velocidade e aceleração ao longo de uma sequência de waypoints usando
polinômios de 5ª ordem (*minimum-snap* simplificado), produzindo uma trajetória
contínua e suave.

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `TrajectoryPlanner_codegen::init()` | Inicializa o planner a partir de `waypoints`, `segmentTimes` e `numSegments`. |
| `TrajectoryPlanner_codegen::getNextSetpoint(t, Xd, Vd, Ad)` | Retorna posição, velocidade e aceleração desejadas no instante `t`. |

### Relação com a FSM / `DroneControllerCompleto`

Chamado em `initialize_trajectory()` para configurar o planner e depois
`getNextSetpoint()` é invocado a cada 10 ms no Estado 3 para fornecer a referência
contínua ao `Drone_codegen`.

### O que cada bloco de código faz

**Bloco: `init()` — cálculo dos coeficientes polinomiais**
```cpp
TrajectoryPlanner_codegen *TrajectoryPlanner_codegen::init() {
  this->currentSegment = 0;
  // para cada segmento: resolve sistema linear 6×6 via mldivide()
  // para obter coeficientes [a0..a5] do polinômio em X, Y e Z
}
```
> Divide a trajetória em `numSegments` segmentos. Para cada segmento, constrói
> e resolve (via `mldivide()`) o sistema de equações que garante posição e
> velocidade contínuas nas junções. Os coeficientes são armazenados internamente
> para consulta em `getNextSetpoint()`.

**Bloco: `getNextSetpoint(t, Xd, Vd, Ad)` — avaliação do polinômio**
```cpp
void TrajectoryPlanner_codegen::getNextSetpoint(double t, double Xd[3], …) {
  // determina o segmento ativo baseado em t
  // avalia polinômio e suas derivadas:
  //   Xd[i] = a0 + a1*tau + a2*tau² + a3*tau³ + a4*tau⁴ + a5*tau⁵
  //   Vd[i] = a1 + 2*a2*tau + …  (1ª derivada)
  //   Ad[i] = 2*a2 + 6*a3*tau + … (2ª derivada)
}
```
> `tau` é o tempo local dentro do segmento atual (`t − t_inicio_segmento`).
> Retorna `(Xd, Vd, Ad)` para as 3 coordenadas (X, Y, Z), prontos para serem
> alimentados ao `Drone_codegen::PositionCtrl()`.

---

## 12. `cos.cpp`

### Papel / Responsabilidade

Arquivo *stub* gerado pelo MATLAB Coder como wrapper da função `std::cos`. No
estado atual do repositório o arquivo está **vazio** (0 bytes): o compilador
resolve `cos()` diretamente da `<cmath>` padrão, tornando este arquivo um
artefato sem conteúdo funcional.

### Principais funções / classes

*Arquivo vazio — nenhuma função implementada.*

### Relação com a FSM / `DroneControllerCompleto`

Referenciado pelo cabeçalho `cos.h` incluído em `Drone_codegen.cpp`. Em tempo de
compilação não contribui com nenhum símbolo. Mantido para compatibilidade com o
*build system* `CMakeLists.txt` gerado pelo MATLAB Coder.

### O que cada bloco de código faz

*Sem blocos — arquivo vazio.*

---

## 13. `minOrMax.cpp`

### Papel / Responsabilidade

Implementa funções auxiliares de mínimo e máximo com limites fixos, geradas pelo
MATLAB Coder. Usadas internamente pelo `TrajectoryPlanner_codegen` para saturar
velocidades e acelerações dentro do envelope configurado (±12 m/s ou m/s²).

### Principais funções / classes

| Função | O que faz |
|--------|-----------|
| `coder::internal::maximum2(x[3], ex[3])` | Aplica `max(xi, −12.0)` em cada componente do vetor. |
| `coder::internal::maximum2(x, y)` | Retorna `max(x, y)` para escalares. |
| `coder::internal::minimum2(x[3], ex[3])` | Aplica `min(xi, +12.0)` em cada componente do vetor. |
| `coder::internal::minimum2(x, y)` | Retorna `min(x, y)` para escalares. |

### Relação com a FSM / `DroneControllerCompleto`

Indiretamente relacionado ao Estado 3: chamado dentro de `TrajectoryPlanner_codegen`
e/ou `Drone_codegen` para saturar saídas. Não acessa diretamente nenhuma variável
do nó.

### O que cada bloco de código faz

**Bloco: `maximum2` vetorial**
```cpp
ex[0] = std::fmax(x[0], -12.0);
ex[1] = std::fmax(x[1], -12.0);
ex[2] = std::fmax(x[2], -12.0);
```
> Garante que nenhuma componente do vetor seja menor que −12,0 (limita inferiormente
> velocidades/acelerações geradas pelo planner).

**Bloco: `minimum2` vetorial**
```cpp
ex[0] = std::fmin(x[0], 12.0);
ex[1] = std::fmin(x[1], 12.0);
ex[2] = std::fmin(x[2], 12.0);
```
> Limita superiormente em +12,0. Combinado com `maximum2`, cria a saturação
> simétrica `[−12, +12]` para cada componente.

---

## 14. `mldivide.cpp`

### Papel / Responsabilidade

Implementa solvers de sistemas lineares por fatoração LU com pivoteamento parcial,
gerados pelo MATLAB Coder. Usados pelo `TrajectoryPlanner_codegen` para calcular
os coeficientes polinomiais de cada segmento da trajetória durante `init()`.

### Principais funções / classes

| Função | Dimensões | O que resolve |
|--------|-----------|---------------|
| `coder::b_mldivide(A[16], B[4])` | 4×4 | Sistema `A·x = B` para 4 incógnitas (segmento simples). |
| `coder::mldivide(A[36], B[6])` | 6×6 | Sistema `A·x = B` para 6 incógnitas (polinômio 5ª ordem). |
| `coder::mldivide(A[9], B[3], Y[3])` | 3×3 | Sistema `A·x = B` para 3 incógnitas. |

### Relação com a FSM / `DroneControllerCompleto`

Chamado apenas durante `TrajectoryPlanner_codegen::init()` (Estado 3, uma única
vez por trajetória). Não influencia diretamente a FSM após a inicialização.

### O que cada bloco de código faz

**Bloco: cópia e inicialização do pivô**
```cpp
std::copy(&A[0], &A[36], &b_A[0]);
for (i = 0; i < 6; i++) ipiv[i] = static_cast<signed char>(i + 1);
```
> Trabalha sobre uma cópia de `A` para não modificar a original. O vetor `ipiv`
> registra as permutações de linha durante o pivoteamento.

**Bloco: eliminação com pivoteamento parcial (loop externo)**
```cpp
for (int j{0}; j < 5; j++) {
  // encontra o maior elemento na coluna j (pivô)
  // troca linha j com a linha do pivô
  // divide a subcoluna pelo pivô
  // atualiza a submatriz abaixo (eliminação)
}
```
> Fatoração LU: constrói `L` (implícito em `b_A`) e `U` (parte superior de `b_A`)
> com pivoteamento para estabilidade numérica. Aplicado sobre `B` conforme as
> mesmas permutações.

**Bloco: substituição direta e retroativa**
```cpp
// Substituição direta (L·y = B):
for (int k{0}; k < 6; k++) { … }
// Substituição retroativa (U·x = y):
for (int k{5}; k >= 0; k--) { B[k] /= b_A[k + jA]; … }
```
> Resolve `L·y = B` (substituição direta, L triangular inferior unitária) e
> depois `U·x = y` (retroativa, U triangular superior). O resultado em `B` é a
> solução `x` do sistema original.

---

## 15. `rt_nonfinite.cpp`

### Papel / Responsabilidade

Inicializa as constantes globais de ponto flutuante (`rtNaN`, `rtInf`,
`rtMinusInf`) usadas pela biblioteca padrão do MATLAB Coder para representar
NaN e infinito em arquiteturas que não oferecem esses literais diretamente.

### Principais funções / classes

*Não define funções — apenas inicializa variáveis globais.*

| Variável | Tipo | Valor |
|----------|------|-------|
| `rtNaN` | `real_T` (double) | `quiet_NaN` |
| `rtInf` | `real_T` (double) | `+∞` |
| `rtMinusInf` | `real_T` (double) | `−∞` |
| `rtNaNF` | `real32_T` (float) | `quiet_NaN` |
| `rtInfF` | `real32_T` (float) | `+∞` |
| `rtMinusInfF` | `real32_T` (float) | `−∞` |

### Relação com a FSM / `DroneControllerCompleto`

Incluído por `Drone_codegen.cpp` e `TrajectoryPlanner_codegen.cpp`. Usado em
verificações de sanidade numéricas internas ao código gerado (e.g., comparações
com `rtNaN` para detectar resultados inválidos).

### O que cada bloco de código faz

**Bloco único — inicialização das constantes**
```cpp
real_T rtNaN     = std::numeric_limits<real_T>::quiet_NaN();
real_T rtInf     = std::numeric_limits<real_T>::infinity();
real_T rtMinusInf = -std::numeric_limits<real_T>::infinity();
real32_T rtNaNF  = std::numeric_limits<real32_T>::quiet_NaN();
real32_T rtInfF  = std::numeric_limits<real32_T>::infinity();
real32_T rtMinusInfF = -std::numeric_limits<real32_T>::infinity();
```
> Usa `<limits>` C++ padrão para obter as representações de NaN e infinito de
> forma portável (funciona em x86, ARM e outras arquiteturas que suportam IEEE 754).
> As variáveis são *extern* no cabeçalho `rt_nonfinite.h` e acessíveis em todo o
> código gerado.

---

## 16. `sqrt.cpp`

### Papel / Responsabilidade

Arquivo *stub* gerado pelo MATLAB Coder como wrapper da função `std::sqrt`. No
estado atual do repositório o arquivo está **vazio** (0 bytes): o compilador
resolve `sqrt()` diretamente da `<cmath>` padrão, tornando este arquivo um
artefato sem conteúdo funcional.

### Principais funções / classes

*Arquivo vazio — nenhuma função implementada.*

### Relação com a FSM / `DroneControllerCompleto`

Referenciado pelo cabeçalho `sqrt.h` incluído em `Drone_codegen.cpp`. Em tempo de
compilação não contribui com nenhum símbolo. Mantido para compatibilidade com o
*build system* gerado pelo MATLAB Coder.

### O que cada bloco de código faz

*Sem blocos — arquivo vazio.*

---

## Tabela-resumo: relação com a FSM

| Arquivo | Estado FSM | Transições que desencadeia |
|---------|-----------|---------------------------|
| `main.cpp` | — | Inicia o nó (pré-FSM) |
| `drone_controller_completo.cpp` | Todos | Dispatcher; callbacks setam `state_voo_` |
| `fsm_state0_wait.cpp` | **0** | Nenhuma (aguarda callback externa) |
| `fsm_takeoff.cpp` | **1** | `1 → 2` (altitude atingida) |
| `fsm_hover.cpp` | **2** | `2 → 3` (`controlador_ativo_`); `2 → 4` (pouso) |
| `fsm_trajectory.cpp` | **3** | `3 → 2` (trajetória concluída); `3 → 4` (pouso) |
| `fsm_landing.cpp` | **4** | `4 → 0` (DISARM confirmado) |
| `command_queue.cpp` | Todos | Auditoria; sem transições diretas |
| `waypoint_validation.cpp` | — | Bloqueia transições inválidas (rejeita waypoints) |
| `Drone_codegen.cpp` | **3** | Sem transições; fornece `zdot_des` |
| `TrajectoryPlanner_codegen.cpp` | **3** | Sem transições; fornece `(Xd, Vd, Ad)` |
| `cos.cpp` | — | Artefato vazio |
| `minOrMax.cpp` | **3** (indireto) | Sem transições; satura saídas do planner |
| `mldivide.cpp` | **3** (init) | Sem transições; resolve coeficientes polinomiais |
| `rt_nonfinite.cpp` | — | Constantes globais; sem lógica de controle |
| `sqrt.cpp` | — | Artefato vazio |
