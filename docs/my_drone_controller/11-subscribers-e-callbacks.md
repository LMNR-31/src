# 11 — Subscribers e Callbacks

> **Arquivo-fonte principal:** `my_drone_controller/src/drone_controller_completo.cpp`  
> **Cabeçalho:** `my_drone_controller/include/my_drone_controller/drone_controller_completo.hpp`

Este documento descreve **todos os subscribers** do nó `DroneControllerCompleto`, o que cada callback faz internamente, quais variáveis de estado são alteradas e como cada mensagem recebida impacta a **Máquina de Estados Finitos (FSM)**.

---

## 1. Tabela Geral de Subscribers

| Membro da classe | Tópico (ROS 2) | Tipo de Mensagem | QoS depth | Função Callback |
|---|---|---|---|---|
| `state_sub_` | `/uav1/mavros/state` | `mavros_msgs/msg/State` | 10 | lambda inline |
| `extended_state_sub_` | `/uav1/mavros/extended_state` | `mavros_msgs/msg/ExtendedState` | 10 | lambda inline |
| `waypoints_sub_` | `waypoints_cmd_topic_` *(parâmetro)* | `geometry_msgs/msg/PoseArray` | **1** | `waypoints_callback` |
| `waypoint_goal_sub_` | `waypoint_goal_cmd_topic_` *(parâmetro)* | `geometry_msgs/msg/PoseStamped` | **1** | `waypoint_goal_callback` |
| `odom_sub_` | `/uav1/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | 10 | `odometry_callback` |
| `yaw_override_sub_` | `/uav1/yaw_override/cmd` | `drone_control/msg/YawOverride` | 10 | `yaw_override_callback` |
| `waypoint_goal_4d_sub_` | `/waypoint_goal_4d` | `drone_control/msg/Waypoint4D` | **1** | `waypoint_goal_4d_callback` |
| `waypoints_4d_sub_` | `/waypoints_4d` | `drone_control/msg/Waypoint4DArray` | **1** | `waypoints_4d_callback` |

> **Nota sobre QoS depth=1:** Os tópicos de comando de waypoints usam profundidade 1 intencionalmente — apenas o comando mais recente é mantido na fila. Isso evita que comandos antigos (acumulados enquanto o controlador estava indisponível) sejam processados após a reconexão.

---

## 2. Criação dos Subscribers (`setup_subscribers`)

```cpp
// drone_controller_completo.cpp — linhas ~177-220
void DroneControllerCompleto::setup_subscribers()
{
  // 1. Estado do FCU (modo de voo, armed)
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/uav1/mavros/state", 10,
    [this](const mavros_msgs::msg::State::SharedPtr msg) {
      current_state_ = *msg;   // cópia simples: armed, mode, connected
    });

  // 2. Estado estendido (landed_state: ONGROUND, LANDING, TAKEOFF, INFLIGHT)
  extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
    "/uav1/mavros/extended_state", 10,
    [this](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
      last_extended_state_      = *msg;
      last_extended_state_time_ = this->now();   // timestamp de frescor
      extended_state_received_  = true;           // flag "já recebemos pelo menos 1"
    });

  // 3. Lista de waypoints 3D (trajetória ou takeoff/pouso)
  waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    waypoints_cmd_topic_, 1,
    std::bind(&DroneControllerCompleto::waypoints_callback, this, std::placeholders::_1));

  // 4. Waypoint-goal único 3D (hover ou redirecionamento de trajetória)
  waypoint_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    waypoint_goal_cmd_topic_, 1,
    std::bind(&DroneControllerCompleto::waypoint_goal_callback, this, std::placeholders::_1));

  // 5. Odometria local (posição + velocidade + yaw atuais do drone)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/uav1/mavros/local_position/odom", 10,
    std::bind(&DroneControllerCompleto::odometry_callback, this, std::placeholders::_1));

  // 6. Comando de yaw-rate externo (congelamento de posição + rotação)
  yaw_override_sub_ = this->create_subscription<drone_control::msg::YawOverride>(
    "/uav1/yaw_override/cmd", 10,
    std::bind(&DroneControllerCompleto::yaw_override_callback, this, std::placeholders::_1));

  // 7. Waypoint-goal único 4D (X, Y, Z + yaw explícito)
  waypoint_goal_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4D>(
    "/waypoint_goal_4d", 1,
    std::bind(&DroneControllerCompleto::waypoint_goal_4d_callback, this, std::placeholders::_1));

  // 8. Lista de waypoints 4D (trajetória com yaw por ponto)
  waypoints_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4DArray>(
    "/waypoints_4d", 1,
    std::bind(&DroneControllerCompleto::waypoints_4d_callback, this, std::placeholders::_1));
}
```

---

## 3. Guardas Anti-Echo

### O problema

O `DroneControllerCompleto` **publica de volta** em alguns tópicos como mecanismo de monitoramento (heartbeat de status). Quando os parâmetros `waypoints_cmd_topic_` e `waypoints_status_topic_` apontam para o **mesmo tópico**, o nó receberia sua própria mensagem publicada, criando um loop de auto-processamento.

### A solução: contadores de descarte

```cpp
// drone_controller_completo.hpp
int skip_self_waypoint_goal_count_{0};  // contador de descartes pendentes para waypoint_goal
int skip_self_waypoints_count_{0};      // contador de descartes pendentes para waypoints
```

**Funcionamento:**

```
publish_waypoint_goal_status()          publish_waypoints_status()
        |                                       |
        | (se cmd_topic == status_topic)        | (se cmd_topic == status_topic)
        ▼                                       ▼
  skip_self_waypoint_goal_count_++      skip_self_waypoints_count_++
        |                                       |
        | (publica no tópico)                   | (publica no tópico)
        ▼                                       ▼
  waypoint_goal_callback dispara       waypoints_callback dispara
        |                                       |
        ▼                                       ▼
  if (skip_self_waypoint_goal_count_ > 0)    if (skip_self_waypoints_count_ > 0)
    { skip_self_waypoint_goal_count_--;        { skip_self_waypoints_count_--;
      return; }  // mensagem descartada          return; }  // mensagem descartada
```

**Onde o contador é incrementado:**

| Função que publica | Condição de incremento | Contador |
|---|---|---|
| `publish_waypoint_goal_status()` | `waypoint_goal_cmd_topic_ == waypoint_goal_status_topic_` | `skip_self_waypoint_goal_count_++` |
| `monitor_waypoint_goal_heartbeat()` | idem | `skip_self_waypoint_goal_count_++` |
| `publish_waypoints_status()` | `waypoints_cmd_topic_ == waypoints_status_topic_` | `skip_self_waypoints_count_++` |
| `monitor_waypoints_heartbeat()` | idem | `skip_self_waypoints_count_++` |

> **Importante:** Em instalações onde os tópicos de comando e status são distintos (ex.: `/waypoints_cmd` vs `/waypoints_status`), os contadores permanecem sempre 0 e nenhuma mensagem é descartada.

---

## 4. Callback: `/uav1/mavros/state`

**Tipo:** `mavros_msgs/msg/State`  
**QoS:** depth 10  
**Callback:** lambda inline em `setup_subscribers`

```cpp
[this](const mavros_msgs::msg::State::SharedPtr msg) {
  current_state_ = *msg;
}
```

### Campos atualizados

| Campo da mensagem | Variável interna | Tipo | Descrição |
|---|---|---|---|
| `msg->armed` | `current_state_.armed` | `bool` | `true` se FCU está armado |
| `msg->mode` | `current_state_.mode` | `string` | ex.: `"OFFBOARD"`, `"MANUAL"` |
| `msg->connected` | `current_state_.connected` | `bool` | se MAVLink está conectado |

### Efeitos na FSM

A variável `current_state_` é **consultada** (não modifica a FSM diretamente):

- Em `fsm_takeoff.cpp` → `handle_state1_takeoff()`: verifica se `current_state_.mode == "OFFBOARD"` para confirmar `offboard_mode_confirmed_` e então enviar ARM.
- Em `fsm_takeoff.cpp`: verifica `current_state_.armed` para confirmar `activation_confirmed_`.
- Em `fsm_landing.cpp` → `handle_state4_landing()`: verifica `current_state_.armed` para determinar se DISARM foi aceito pelo FCU.

### Exemplo de publicação C++

```cpp
// Exemplo: simular confirmação de modo OFFBOARD + ARM (para testes)
#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

auto node = rclcpp::Node::make_shared("test_state_pub");
auto pub = node->create_publisher<mavros_msgs::msg::State>("/uav1/mavros/state", 10);

mavros_msgs::msg::State msg;
msg.header.stamp = node->now();
msg.connected     = true;
msg.armed         = true;
msg.mode          = "OFFBOARD";
pub->publish(msg);
```

---

## 5. Callback: `/uav1/mavros/extended_state`

**Tipo:** `mavros_msgs/msg/ExtendedState`  
**QoS:** depth 10  
**Callback:** lambda inline em `setup_subscribers`

```cpp
[this](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
  last_extended_state_      = *msg;          // cópia completa da mensagem
  last_extended_state_time_ = this->now();   // registra instante de chegada
  extended_state_received_  = true;          // flag: já recebemos ao menos 1
}
```

### Campos atualizados

| Variável interna | Tipo | O que representa |
|---|---|---|
| `last_extended_state_` | `mavros_msgs::msg::ExtendedState` | cópia completa da última mensagem |
| `last_extended_state_time_` | `rclcpp::Time` | instante ROS em que a mensagem chegou |
| `extended_state_received_` | `bool` | latch: torna-se `true` na 1ª mensagem, nunca volta a `false` |

### Efeitos na FSM via `autopilot_indicates_landing()`

A função auxiliar `autopilot_indicates_landing()` usa os três valores acima para decidir se o autopiloto indica pouso:

```cpp
bool DroneControllerCompleto::autopilot_indicates_landing() const
{
  if (!extended_state_received_) { return false; }          // nunca recebemos → ignora
  if ((this->now() - last_extended_state_time_).seconds() > 1.0) { return false; } // mensagem velha
  if (!current_state_.armed) { return false; }              // drone desarmado → ignora

  const uint8_t ls = last_extended_state_.landed_state;
  return (ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING ||
          ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND);
}
```

Isso é chamado em:
- `check_landing_in_flight()` → dispara `trigger_landing()` se drone em estado 2 ou 3.
- `detect_and_handle_landing_in_trajectory()` → idem durante trajetória.

### Exemplo de publicação C++

```cpp
#include "mavros_msgs/msg/extended_state.hpp"
#include "rclcpp/rclcpp.hpp"

auto node = rclcpp::Node::make_shared("test_ext_state");
auto pub = node->create_publisher<mavros_msgs::msg::ExtendedState>(
  "/uav1/mavros/extended_state", 10);

mavros_msgs::msg::ExtendedState msg;
msg.header.stamp  = node->now();
// LANDED_STATE_ON_GROUND = 1, LANDING = 4, IN_AIR = 3
msg.landed_state  = mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
pub->publish(msg);
```

---

## 6. Callback: `odometry_callback`

**Tópico:** `/uav1/mavros/local_position/odom`  
**Tipo:** `nav_msgs/msg/Odometry`  
**QoS:** depth 10

```cpp
void DroneControllerCompleto::odometry_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);       // protege acesso concorrente

  // Posição local (NED — North/East/Down no frame do mapa)
  current_x_ned_ = msg->pose.pose.position.x;
  current_y_ned_ = msg->pose.pose.position.y;
  current_z_ned_ = msg->pose.pose.position.z;
  current_z_real_ = std::abs(current_z_ned_);    // altitude positiva (|z|)

  // Velocidade linear local
  current_vx_ned_ = msg->twist.twist.linear.x;
  current_vy_ned_ = msg->twist.twist.linear.y;
  current_vz_ned_ = msg->twist.twist.linear.z;

  // Yaw extraído do quaternion da odometria
  const auto & q = msg->pose.pose.orientation;
  current_yaw_rad_ = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
```

### Variáveis atualizadas

| Variável | Tipo | Fonte |
|---|---|---|
| `current_x_ned_` | `double` | `pose.pose.position.x` |
| `current_y_ned_` | `double` | `pose.pose.position.y` |
| `current_z_ned_` | `double` | `pose.pose.position.z` (pode ser negativo em NED) |
| `current_z_real_` | `double` | `abs(current_z_ned_)` — altitude absoluta |
| `current_vx_ned_` | `double` | `twist.twist.linear.x` |
| `current_vy_ned_` | `double` | `twist.twist.linear.y` |
| `current_vz_ned_` | `double` | `twist.twist.linear.z` |
| `current_yaw_rad_` | `double` | atan2 do quaternion (radianos, [-π, π]) |

### Efeitos na FSM

Esta callback **não altera `state_voo_` diretamente**, mas fornece os valores usados em todo o loop de controle:

- `current_z_real_` → detectar altitude de hover (`handle_state1_takeoff`).
- `current_x_ned_`, `current_y_ned_`, `current_z_ned_` → erro de posição no controlador de trajetória.
- `current_yaw_rad_` → cálculo de yaw-follow na `fsm_trajectory.cpp`.
- `current_x_ned_/current_y_ned_` → capturado como `hold_x/y_ned_` ao ativar yaw override.

### Exemplo de publicação C++

```cpp
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

auto node = rclcpp::Node::make_shared("test_odom");
auto pub = node->create_publisher<nav_msgs::msg::Odometry>(
  "/uav1/mavros/local_position/odom", 10);

nav_msgs::msg::Odometry msg;
msg.header.stamp    = node->now();
msg.header.frame_id = "odom";

// Drone a 2.5 m de altitude, posição X=1.0 Y=-0.5
msg.pose.pose.position.x = 1.0;
msg.pose.pose.position.y = -0.5;
msg.pose.pose.position.z = 2.5;   // Z positivo = altitude em ENU (MAVROS converte para NED)

// Orientação: quaternion identidade (yaw = 0)
msg.pose.pose.orientation.w = 1.0;

msg.twist.twist.linear.x = 0.1;   // velocidade X em m/s
msg.twist.twist.linear.y = 0.0;
msg.twist.twist.linear.z = 0.05;

pub->publish(msg);
```

---

## 7. Callback: `yaw_override_callback`

**Tópico:** `/uav1/yaw_override/cmd`  
**Tipo:** `drone_control/msg/YawOverride`  
**QoS:** depth 10

```cpp
void DroneControllerCompleto::yaw_override_callback(
  const drone_control::msg::YawOverride::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->enable) {
    if (!yaw_override_enabled_) {
      // Primeira ativação: captura posição atual como ponto de hold
      hold_x_ned_ = current_x_ned_;
      hold_y_ned_ = current_y_ned_;
      hold_z_ned_ = current_z_ned_;
      hold_valid_  = true;
      RCLCPP_INFO(..., "🔒 Yaw override ATIVADO...");
    }
    yaw_override_enabled_ = true;
    yaw_rate_cmd_         = static_cast<double>(msg->yaw_rate);
    if (msg->timeout > 0.0f) {
      yaw_override_timeout_s_ = static_cast<double>(msg->timeout);
    }
  } else {
    if (yaw_override_enabled_) {
      RCLCPP_INFO(..., "🔓 Yaw override DESATIVADO...");
    }
    yaw_override_enabled_ = false;
    yaw_rate_cmd_          = 0.0;
  }
  last_yaw_cmd_time_ = this->now();   // reinicia o watchdog de timeout
}
```

### Variáveis atualizadas

| Variável | Tipo | Significado |
|---|---|---|
| `yaw_override_enabled_` | `bool` | `true` = posição congelada + yaw_rate ativo |
| `yaw_rate_cmd_` | `double` | taxa de rotação em rad/s |
| `yaw_override_timeout_s_` | `double` | duração máxima do override (0 = sem timeout) |
| `last_yaw_cmd_time_` | `rclcpp::Time` | instante da última mensagem (watchdog) |
| `hold_x/y/z_ned_` | `double` | posição NED congelada (somente na 1ª ativação) |
| `hold_valid_` | `bool` | indica que `hold_x/y/z` têm valores válidos |

### Efeito na FSM

O yaw override **suspende** o despacho normal da FSM dentro do `control_loop()`:

```
control_loop() a cada 10 ms (100 Hz)
  │
  ├─ watchdog: (now - last_yaw_cmd_time_) > yaw_override_timeout_s_
  │    └─ se expirado: yaw_override_enabled_ = false  (desativa automaticamente)
  │
  └─ if (yaw_override_enabled_)
       └─ publica setpoint de hold (hold_x/y/z) + yaw_rate
          e RETORNA — a FSM (state 0..4) NÃO é chamada neste ciclo
```

O estado `state_voo_` **não é alterado** pelo yaw override. Quando `enable=false` chega, a FSM retoma exatamente no estado onde parou.

### Estrutura da mensagem `YawOverride`

```cpp
// drone_control/msg/YawOverride.msg
bool   enable      # true = ativar override, false = desativar
float32 yaw_rate   # velocidade de rotação em rad/s (+ = anti-horário vista de cima)
float32 timeout    # duração máxima em segundos; 0 = sem limite automático
```

### Exemplo de publicação C++

```cpp
#include "drone_control/msg/yaw_override.hpp"
#include "rclcpp/rclcpp.hpp"

auto node = rclcpp::Node::make_shared("test_yaw_override");
auto pub = node->create_publisher<drone_control::msg::YawOverride>(
  "/uav1/yaw_override/cmd", 10);

// Ativar: girar a 0.5 rad/s por 3 segundos
drone_control::msg::YawOverride msg;
msg.enable   = true;
msg.yaw_rate = 0.5f;   // rad/s
msg.timeout  = 3.0f;   // segundos (0 = sem timeout automático)
pub->publish(msg);

// Desativar manualmente (antes do timeout)
msg.enable   = false;
msg.yaw_rate = 0.0f;
pub->publish(msg);
```

---

## 8. Callback: `waypoint_goal_callback`

**Tópico:** `waypoint_goal_cmd_topic_` *(padrão: `/uav1/waypoint_goal/cmd`)*  
**Tipo:** `geometry_msgs/msg/PoseStamped`  
**QoS:** depth 1

```cpp
void DroneControllerCompleto::waypoint_goal_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // ① Guarda anti-echo
  if (skip_self_waypoint_goal_count_ > 0) { skip_self_waypoint_goal_count_--; return; }

  // ② Modo 3D (sem yaw explícito)
  using_4d_goal_ = false;

  // ③ Validação de segurança
  if (!validate_waypoint(*msg, config_)) {
    RCLCPP_WARN(..., "❌ waypoint_goal inválido ...");
    return;
  }

  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  last_z_ = z;

  // ④ Detecção de pouso em voo (via ExtendedState)
  if (check_landing_in_flight(z)) { return; }

  // ⑤ Tratamento de estado 4 pendente (DISARM em andamento)
  if (handle_state4_disarm_reset()) { return; }

  // ⑥ Redirecionamento durante trajetória (estado 3)
  if (state_voo_ == 3) {
    trajectory_setpoint_[0] = x;
    trajectory_setpoint_[1] = y;
    trajectory_setpoint_[2] = z;
    controlador_ativo_    = true;
    pouso_em_andamento_   = false;
    publish_waypoint_goal_status(x, y, z);
    return;
  }

  // ⑦ Novo waypoint de hover / decolagem
  last_waypoint_goal_      = *msg;
  waypoint_goal_received_  = true;
  controlador_ativo_        = true;
  pouso_em_andamento_       = false;
  publish_waypoint_goal_status(x, y, z);
}
```

### Fluxo de decisão

```
Mensagem recebida em waypoint_goal_cmd_topic_
  │
  ├─ skip_self_waypoint_goal_count_ > 0  → descarta (anti-echo)
  ├─ !validate_waypoint()                → descarta (NaN/Inf/fora de bounds)
  ├─ check_landing_in_flight()           → trigger_landing() + state_voo_=4
  ├─ handle_state4_disarm_reset()        → aguarda DISARM (estado 4 pendente)
  ├─ state_voo_ == 3                     → atualiza trajectory_setpoint_[] (redirecionamento online)
  └─ caso geral                          → last_waypoint_goal_ + waypoint_goal_received_=true
                                           controlador_ativo_=true, pouso_em_andamento_=false
```

### Variáveis atualizadas

| Variável | Alteração | Condição |
|---|---|---|
| `using_4d_goal_` | `false` | sempre (modo 3D) |
| `last_z_` | valor de `z` | sempre (após validação) |
| `trajectory_setpoint_[0..2]` | `x, y, z` | somente se `state_voo_ == 3` |
| `controlador_ativo_` | `true` | se redirecionamento ou goal normal |
| `pouso_em_andamento_` | `false` | se redirecionamento ou goal normal |
| `last_waypoint_goal_` | cópia do msg | goal normal (estados 0, 1, 2) |
| `waypoint_goal_received_` | `true` | goal normal |
| `state_voo_` | `4` | via `trigger_landing()` se pouso detectado |

### Exemplo de publicação C++

```cpp
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

auto node = rclcpp::Node::make_shared("test_waypoint_goal");
auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
  "/uav1/waypoint_goal/cmd", 1);

geometry_msgs::msg::PoseStamped msg;
msg.header.stamp    = node->now();
msg.header.frame_id = "map";
msg.pose.position.x = 3.0;    // X em metros (NED local)
msg.pose.position.y = 1.5;
msg.pose.position.z = 2.0;    // altitude de hover desejada

// Orientação: identidade (yaw não é usado neste subscriber)
msg.pose.orientation.w = 1.0;

pub->publish(msg);
```

---

## 9. Callback: `waypoint_goal_4d_callback`

**Tópico:** `/waypoint_goal_4d`  
**Tipo:** `drone_control/msg/Waypoint4D`  
**QoS:** depth 1

```cpp
void DroneControllerCompleto::waypoint_goal_4d_callback(
  const drone_control::msg::Waypoint4D::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // ① Validação
  geometry_msgs::msg::PoseStamped ps; ps.pose = msg->pose;
  if (!validate_waypoint(ps, config_)) { ... return; }

  // ② Normalização do yaw
  if (std::isnan(static_cast<double>(msg->yaw))) {
    goal_yaw_rad_ = current_yaw_rad_;               // mantém yaw atual
  } else {
    double raw_yaw = static_cast<double>(msg->yaw);
    goal_yaw_rad_ = std::atan2(std::sin(raw_yaw), std::cos(raw_yaw));  // normaliza para [-π,π]
  }

  // ③ Flags de modo
  using_4d_goal_     = true;
  trajectory_4d_mode_ = false;

  // ④ Mesmo fluxo de decisão do 3D
  // check_landing_in_flight / handle_state4_disarm_reset /
  // redirecionamento em estado 3 / goal normal
}
```

### Diferenças em relação ao `waypoint_goal_callback` (3D)

| Aspecto | 3D (`PoseStamped`) | 4D (`Waypoint4D`) |
|---|---|---|
| Yaw explícito | Não — yaw não é lido | Sim — campo `yaw` (rad, NaN = manter atual) |
| `using_4d_goal_` | `false` | `true` |
| `goal_yaw_rad_` | não alterado | atualizado (ou mantém `current_yaw_rad_` se NaN) |
| Comportamento em hover | yaw seguindo waypoint anterior | yaw seguindo `goal_yaw_rad_` |

### Estrutura da mensagem `Waypoint4D`

```cpp
// drone_control/msg/Waypoint4D.msg
geometry_msgs/Pose pose   # posição X, Y, Z + orientação (apenas XYZ é usado)
float32 yaw               # ângulo de yaw em rad; NaN = manter yaw atual
```

### Exemplo de publicação C++

```cpp
#include "drone_control/msg/waypoint4_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

auto node = rclcpp::Node::make_shared("test_wp4d_goal");
auto pub = node->create_publisher<drone_control::msg::Waypoint4D>(
  "/waypoint_goal_4d", 1);

drone_control::msg::Waypoint4D msg;
msg.pose.position.x  = 2.0;
msg.pose.position.y  = -1.0;
msg.pose.position.z  = 3.0;
msg.pose.orientation.w = 1.0;
msg.yaw = static_cast<float>(M_PI / 4.0);  // 45° (norte-leste)
// msg.yaw = std::numeric_limits<float>::quiet_NaN();  // manter yaw atual

pub->publish(msg);
```

---

## 10. Callback: `waypoints_callback`

**Tópico:** `waypoints_cmd_topic_` *(padrão: `/uav1/waypoints/cmd`)*  
**Tipo:** `geometry_msgs/msg/PoseArray`  
**QoS:** depth 1

Esta é a callback mais complexa: rota comandos de **takeoff**, **trajetória** e **pouso** dependendo do número de poses e do valor de Z.

```cpp
void DroneControllerCompleto::waypoints_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // ① Guarda anti-echo
  if (skip_self_waypoints_count_ > 0) { skip_self_waypoints_count_--; return; }

  // ② Modo 3D
  trajectory_4d_mode_ = false;

  // ③ Mínimo de 1 pose
  if (msg->poses.size() < 1) { RCLCPP_WARN(...); return; }

  // ④ Validação de todas as poses
  for (size_t i = 0; i < msg->poses.size(); ++i) {
    if (!validate_pose(msg->poses[i], config_)) { ... return; }
  }

  double last_z = msg->poses.back().position.z;

  // ⑤ 1 pose com Z < land_z_threshold → POUSO
  if (msg->poses.size() == 1 && last_z < config_.land_z_threshold) {
    handle_landing_waypoint_command(last_z);  // → trigger_landing() + state_voo_=4
    return;
  }

  // ⑥ 1 pose com Z >= land_z_threshold e drone em solo → DECOLAGEM
  if (msg->poses.size() == 1 && last_z >= config_.land_z_threshold && !is_in_flight()) {
    if (handle_state4_disarm_reset()) { return; }  // espera DISARM se estado 4
    handle_single_takeoff_waypoint_command(msg->poses[0]);  // → state_voo_=1
    return;
  }

  // ⑦ 2+ poses (ou 1 pose durante voo) → TRAJETÓRIA
  if (msg->poses.size() >= 2 || (msg->poses.size() == 1 && is_in_flight())) {
    if (state_voo_ == 4) { RCLCPP_WARN(...); return; }

    trajectory_waypoints_         = msg->poses;
    trajectory_yaws_.clear();
    current_waypoint_idx_          = 0;
    trajectory_started_            = false;
    last_waypoint_reached_idx_     = -1;
    last_waypoint_goal_.pose       = msg->poses[0];

    publish_waypoints_status();

    if (state_voo_ != 2) {
      controlador_ativo_    = false;
      pouso_em_andamento_   = false;
      return;  // trajetória armazenada; será ativada ao entrar em HOVER
    }
    activate_trajectory_in_hover(msg->poses.size());  // → state_voo_=3
  }
}
```

### Árbol de Decisão

```
PoseArray recebida
  │
  ├─ anti-echo check           → descarta se self-echo
  ├─ poses.size() < 1          → descarta
  ├─ validate_pose falha       → descarta
  │
  ├─ size==1, z < threshold    → POUSO   (trigger_landing, state_voo_=4)
  ├─ size==1, z≥threshold,
  │  !is_in_flight()           → TAKEOFF (state_voo_=1)
  │
  └─ size≥2 OU (size==1 e em voo)
       ├─ state_voo_==4        → ignorado (pouso em andamento)
       ├─ state_voo_==2        → ativa imediatamente (state_voo_=3)
       └─ outros estados       → armazena (ativado quando chegar ao HOVER)
```

### Variáveis atualizadas (ramo trajetória)

| Variável | Alteração |
|---|---|
| `trajectory_4d_mode_` | `false` |
| `trajectory_waypoints_` | cópia de `msg->poses` |
| `trajectory_yaws_` | limpo (sem yaw por ponto) |
| `current_waypoint_idx_` | `0` |
| `trajectory_started_` | `false` |
| `last_waypoint_reached_idx_` | `-1` |
| `last_waypoint_goal_.pose` | `msg->poses[0]` |
| `controlador_ativo_` | `false` (se não ativado imediatamente) |
| `pouso_em_andamento_` | `false` |
| `state_voo_` | `3` (se estava em hover) ou mantém |

### Detectando trajetória de descida automática

Quando exatamente 2 poses chegam e `poses.back().z < land_z_threshold`, o controlador reconhece como **trajetória de descida de pouso** (gerada pela missão automática). Neste caso o log é suprimido (RCLCPP_DEBUG) para não poluir o terminal durante ciclos de missão repetitivos.

### Exemplo de publicação C++

```cpp
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

auto node = rclcpp::Node::make_shared("test_waypoints");
auto pub = node->create_publisher<geometry_msgs::msg::PoseArray>(
  "/uav1/waypoints/cmd", 1);

geometry_msgs::msg::PoseArray msg;
msg.header.stamp    = node->now();
msg.header.frame_id = "map";

// Trajetória simples: 3 waypoints
geometry_msgs::msg::Pose p;
p.orientation.w = 1.0;

p.position = {0.0, 0.0, 2.0};  msg.poses.push_back(p);  // WP0
p.position = {3.0, 0.0, 2.5};  msg.poses.push_back(p);  // WP1
p.position = {3.0, 3.0, 2.5};  msg.poses.push_back(p);  // WP2

pub->publish(msg);
```

---

## 11. Callback: `waypoints_4d_callback`

**Tópico:** `/waypoints_4d`  
**Tipo:** `drone_control/msg/Waypoint4DArray`  
**QoS:** depth 1

Versão 4D da `waypoints_callback`: cada waypoint carrega também um ângulo de yaw. O roteamento (takeoff / trajetória / pouso) é idêntico ao 3D, com as seguintes diferenças:

```cpp
void DroneControllerCompleto::waypoints_4d_callback(
  const drone_control::msg::Waypoint4DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // ① Validação de todos os pontos
  for (...) { if (!validate_waypoint(ps, config_)) { ... return; } }

  // ② Extrai poses e yaws (normaliza yaw para [-π,π]; NaN → current_yaw_rad_)
  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<double> yaws;
  for (const auto & wp : msg->waypoints) {
    poses.push_back(wp.pose);
    if (std::isnan(static_cast<double>(wp.yaw)))
      yaws.push_back(current_yaw_rad_);
    else
      yaws.push_back(std::atan2(std::sin(wp.yaw), std::cos(wp.yaw)));
  }

  // ③ Roteamento (igual ao 3D)
  // size==1 e z<threshold → pouso
  // size==1 e z>=threshold → takeoff 4D (goal_yaw_rad_ = yaws[0])
  // size>=2                → trajetória 4D (trajectory_yaws_ = yaws)
}
```

### Diferenças em relação ao 3D

| Aspecto | `waypoints_callback` (3D) | `waypoints_4d_callback` (4D) |
|---|---|---|
| `trajectory_4d_mode_` | `false` | `true` (somente no ramo trajetória) |
| `trajectory_yaws_` | limpo (vazio) | populado com yaw de cada ponto |
| `goal_yaw_rad_` | não alterado | definido por `yaws[0]` (takeoff 4D) |
| `using_4d_goal_` | não alterado | `true` (takeoff 4D) |
| Guard anti-echo | `skip_self_waypoints_count_` | **não possui** (tópico fixo `/waypoints_4d`) |

> **Nota:** O tópico `/waypoints_4d` é fixo e não compartilha nome com nenhum tópico de status do controlador, por isso não há necessidade do guard anti-echo.

### Estrutura das mensagens 4D

```cpp
// drone_control/msg/Waypoint4D.msg
geometry_msgs/Pose pose   # posição + orientação (apenas position.xyz é usado)
float32 yaw               # ângulo de yaw em radianos; NaN = manter yaw atual

// drone_control/msg/Waypoint4DArray.msg
Waypoint4D[] waypoints    # array de waypoints 4D
```

### Exemplo de publicação C++

```cpp
#include "drone_control/msg/waypoint4_d_array.hpp"
#include "drone_control/msg/waypoint4_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

auto node = rclcpp::Node::make_shared("test_waypoints_4d");
auto pub = node->create_publisher<drone_control::msg::Waypoint4DArray>(
  "/waypoints_4d", 1);

drone_control::msg::Waypoint4DArray msg;

auto make_wp = [](double x, double y, double z, double yaw_deg) {
  drone_control::msg::Waypoint4D wp;
  wp.pose.position.x  = x;
  wp.pose.position.y  = y;
  wp.pose.position.z  = z;
  wp.pose.orientation.w = 1.0;
  wp.yaw = static_cast<float>(yaw_deg * M_PI / 180.0);
  return wp;
};

msg.waypoints.push_back(make_wp(0.0, 0.0, 2.5,   0.0));  // WP0 — frente
msg.waypoints.push_back(make_wp(3.0, 0.0, 2.5,  90.0));  // WP1 — virando 90°
msg.waypoints.push_back(make_wp(3.0, 3.0, 2.5, 180.0));  // WP2 — virando 180°

pub->publish(msg);
```

---

## 12. Efeitos Consolidados na FSM

A tabela abaixo resume **quais transições de `state_voo_` cada callback pode causar**:

| Callback | `state_voo_` antes | Ação | `state_voo_` depois |
|---|---|---|---|
| `waypoints_callback` | 0 | takeoff (size=1, z≥threshold) | **1** |
| `waypoints_callback` | 0, 1, 3 | trajetória (size≥2) | mantém → **3** ao chegar no HOVER |
| `waypoints_callback` | 2 | trajetória imediata (size≥2) | **3** |
| `waypoints_callback` | 2, 3 | pouso (size=1, z<threshold) | **4** |
| `waypoints_4d_callback` | 0 | takeoff 4D (size=1) | **1** |
| `waypoints_4d_callback` | 2 | trajetória 4D imediata | **3** |
| `waypoints_4d_callback` | 2, 3 | pouso (size=1, z<threshold) | **4** |
| `waypoint_goal_callback` | 2, 3 | pouso (ExtendedState: LANDING/ONGROUND) | **4** |
| `waypoint_goal_4d_callback` | 2, 3 | pouso (ExtendedState: LANDING/ONGROUND) | **4** |
| `extended_state_sub_` | 2, 3 | via `autopilot_indicates_landing()` indiretamente | **4** |
| `yaw_override_callback` | qualquer | **não altera** `state_voo_` | mantém |
| `odometry_callback` | qualquer | **não altera** `state_voo_` | mantém |
| `state_sub_` | qualquer | **não altera** `state_voo_` | mantém |

---

## 13. Diagrama de Fluxo de Dados

```
Tópicos externos             Callbacks           Variáveis internas      FSM
══════════════════           ═════════           ══════════════════      ═══

/uav1/mavros/state  ────► lambda inline ──────► current_state_    ─────► (lida pela FSM)

/uav1/mavros/
extended_state      ────► lambda inline ──────► last_extended_state_
                                                last_extended_state_time_
                                                extended_state_received_ ─► autopilot_indicates_landing()

/uav1/mavros/
local_position/odom ────► odometry_callback ──► current_{x,y,z}_ned_
                                                current_{vx,vy,vz}_ned_
                                                current_yaw_rad_   ─────► (lida pela FSM / controlador)

/uav1/yaw_override/
cmd                 ────► yaw_override_callback ► yaw_override_enabled_
                                                  yaw_rate_cmd_
                                                  hold_{x,y,z}_ned_  ──► controle suspende FSM

waypoint_goal_cmd/  ────► waypoint_goal_callback ► last_waypoint_goal_
                   ─────► waypoint_goal_4d_cb    ► waypoint_goal_received_
                                                   goal_yaw_rad_ (4D)
                                                   using_4d_goal_    ──► hover 3D / 4D

waypoints_cmd/      ────► waypoints_callback    ─► trajectory_waypoints_
/waypoints_4d       ────► waypoints_4d_callback ─► trajectory_yaws_
                                                   trajectory_4d_mode_
                                                   state_voo_        ──► FSM transição
                                                   controlador_ativo_
                                                   pouso_em_andamento_
```

---

## 14. Resumo de Variáveis de Flags

| Variável | Tipo | Quem seta `true` | Quem seta `false` | Significado |
|---|---|---|---|---|
| `controlador_ativo_` | `bool` | callbacks de waypoint | FSM pouso/reset | posição-alvo válida |
| `pouso_em_andamento_` | `bool` | `trigger_landing()` | waypoint callbacks | pouso ativo |
| `waypoint_goal_received_` | `bool` | `waypoint_goal_callback` | `init_variables()` | ao menos 1 goal recebido |
| `using_4d_goal_` | `bool` | `waypoint_goal_4d_callback` | `waypoint_goal_callback` | goal tem yaw explícito |
| `trajectory_4d_mode_` | `bool` | `waypoints_4d_callback` | `waypoints_callback` | trajetória tem yaws |
| `yaw_override_enabled_` | `bool` | `yaw_override_callback` | watchdog ou `enable=false` | override ativo |
| `extended_state_received_` | `bool` | `extended_state_sub_` | nunca | latch de 1ª mensagem |
| `offboard_activated_` | `bool` | FSM takeoff | FSM reset | SET_MODE enviado |
| `offboard_mode_confirmed_` | `bool` | FSM takeoff | FSM reset | FCU confirmou OFFBOARD |
| `arm_requested_` | `bool` | FSM takeoff | FSM reset | ARM enviado |
| `activation_confirmed_` | `bool` | FSM takeoff | FSM reset | FCU confirmou ARM |
| `disarm_requested_` | `bool` | FSM landing | FSM reset | DISARM enviado |
