# 03 — `drone_controller_completo.cpp` — Parte 1: Setup

Este documento detalha a primeira metade do arquivo `src/drone_controller_completo.cpp`, cobrindo os blocos de **includes, namespace, construtor, `load_parameters`, `setup_publishers`, `setup_subscribers`, `setup_services`** e **`init_variables`**.

## 1. Includes e Namespace

```cpp
// Arquivo: src/drone_controller_completo.cpp
#include "my_drone_controller/drone_controller_completo.hpp"

#include <cmath>       // std::abs, std::atan2, std::hypot, std::sqrt
#include <functional>  // std::bind (para callbacks de subscriber/timer)
#include <string>      // std::string, std::to_string

using namespace std::chrono_literals;  // habilita "1s", "500ms", etc.

namespace drone_control {
  // Todas as implementações de métodos de DroneControllerCompleto estão aqui
}
```

### Dependências externas incluídas indiretamente via `drone_controller_completo.hpp`

| Header incluído | De onde vem | Para que serve no controlador |
|-----------------|-------------|-------------------------------|
| `rclcpp/rclcpp.hpp` | ROS 2 | `Node`, `Publisher`, `Subscription`, `Timer`, `QoS` |
| `geometry_msgs/msg/pose_stamped.hpp` | ROS 2 std | tipo do waypoint goal (X, Y, Z + orientação) |
| `geometry_msgs/msg/pose_array.hpp` | ROS 2 std | lista de waypoints de trajetória |
| `nav_msgs/msg/odometry.hpp` | ROS 2 std | posição/velocidade locais (NED) do drone |
| `mavros_msgs/srv/set_mode.hpp` | MAVROS | serviço para mudar modo de voo (OFFBOARD) |
| `mavros_msgs/srv/command_bool.hpp` | MAVROS | serviço de ARM/DISARM |
| `mavros_msgs/msg/state.hpp` | MAVROS | estado do FCU (armed, mode) |
| `mavros_msgs/msg/extended_state.hpp` | MAVROS | `landed_state` (LANDING, ON_GROUND) |
| `mavros_msgs/msg/position_target.hpp` | MAVROS | setpoint de posição/velocidade/yaw |
| `std_msgs/msg/bool.hpp` | ROS 2 std | `/trajectory_finished` |
| `std_msgs/msg/float32.hpp` | ROS 2 std | `/trajectory_progress` |
| `std_msgs/msg/int32.hpp` | ROS 2 std | `/waypoint_reached`, `/drone_controller/state_voo` |
| `drone_control/msg/yaw_override.hpp` | pacote `drone_control` | comando de yaw rate externo |
| `drone_control/msg/waypoint4_d.hpp` | pacote `drone_control` | waypoint 4D (X, Y, Z, yaw) |
| `drone_control/msg/waypoint4_d_array.hpp` | pacote `drone_control` | lista de waypoints 4D |
| `drone_config.h` | `my_drone_controller` | struct `DroneConfig` |
| `command_queue.hpp` | `my_drone_controller` | `CommandQueue`, `CommandType` |
| `waypoint_validation.hpp` | `my_drone_controller` | `validate_waypoint`, `validate_pose` |
| `TrajectoryPlanner_codegen.h` | `my_drone_controller` | planner polinomial |
| `Drone_codegen.h` | `my_drone_controller` | controlador PID de posição |

## 2. Construtor

```cpp
DroneControllerCompleto::DroneControllerCompleto()
: Node("drone_controller_completo")   // registra o nó no ROS 2 com este nome
{
  // Banner de inicialização (visível no terminal de log)
  RCLCPP_INFO(this->get_logger(), "╔═══...═╗");
  RCLCPP_INFO(this->get_logger(), "║  🚁 CONTROLADOR INTELIGENTE DE DRONE  ║");
  RCLCPP_INFO(this->get_logger(), "╚═══...═╝\n");

  // Sequência de inicialização obrigatória (ordem importa!)
  load_parameters();    // 1. declara e lê parâmetros ROS 2
  setup_publishers();   // 2. cria todos os publishers
  setup_subscribers();  // 3. cria todos os subscribers
  setup_services();     // 4. cria service clients + timer principal
  init_variables();     // 5. inicializa variáveis de estado da FSM

  // Log do estado inicial
  RCLCPP_INFO(this->get_logger(), "📊 STATUS INICIAL:");
  RCLCPP_INFO(this->get_logger(), "   Estado: %d", state_voo_);
  RCLCPP_INFO(this->get_logger(), "   Controlador: %s",
    controlador_ativo_ ? "ATIVO" : "INATIVO");
}
```

> **Por que a ordem importa?** `setup_subscribers` cria callbacks que acessam `config_` (inicializado por `load_parameters`). `setup_services` cria o timer que chama `control_loop()`, que usa publishers criados por `setup_publishers`. Qualquer reordenação pode causar acesso a ponteiros nulos.

## 3. `load_parameters()`

Declara e lê cada parâmetro ROS 2, populando a struct `config_` e variáveis de controle.

```cpp
void DroneControllerCompleto::load_parameters()
{
  // Padrão: usa os valores defaults da struct DroneConfig
  this->declare_parameter("hover_altitude",        config_.hover_altitude);
  this->declare_parameter("hover_altitude_margin", config_.hover_altitude_margin);
  this->declare_parameter("max_altitude",          config_.max_altitude);
  this->declare_parameter("min_altitude",          config_.min_altitude);
  this->declare_parameter("waypoint_duration",     config_.waypoint_duration);
  this->declare_parameter("max_waypoint_distance", config_.max_waypoint_distance);
  this->declare_parameter("land_z_threshold",      config_.land_z_threshold);
  this->declare_parameter("activation_timeout",    config_.activation_timeout);
  this->declare_parameter("command_timeout",       config_.command_timeout);
  this->declare_parameter("landing_timeout",       config_.landing_timeout);
  this->declare_parameter("offboard_confirm_timeout", config_.offboard_confirm_timeout);
  this->declare_parameter("takeoff_z_boost",         config_.takeoff_z_boost);
  this->declare_parameter("takeoff_xy_origin_threshold_m", config_.takeoff_xy_origin_threshold_m);
  this->declare_parameter("latch_pose_max_age_s",          config_.latch_pose_max_age_s);

  // Lê os valores (possivelmente sobrescritos pelo arquivo .yaml)
  config_.hover_altitude        = this->get_parameter("hover_altitude").as_double();
  // ... (repetido para cada parâmetro)

  // Parâmetro booleano: liga/desliga o controlador
  this->declare_parameter<bool>("enabled", true);
  enabled_ = this->get_parameter("enabled").as_bool();

  // Parâmetro booleano: override externo (congela FSM)
  this->declare_parameter<bool>("override_active", false);
  override_active_ = this->get_parameter("override_active").as_bool();

  // Parâmetros de tópicos separados (cmd vs status)
  this->declare_parameter<std::string>("waypoints_cmd_topic",    "/waypoints");
  this->declare_parameter<std::string>("waypoints_status_topic", "/waypoints");
  // ...
  waypoints_cmd_topic_    = this->get_parameter("waypoints_cmd_topic").as_string();
  waypoints_status_topic_ = this->get_parameter("waypoints_status_topic").as_string();

  // Aviso quando cmd e status usam o mesmo tópico
  if (waypoints_cmd_topic_ == waypoints_status_topic_) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ waypoints_cmd_topic == waypoints_status_topic: possível conflito de publisher!");
  }

  // Registra o callback de atualização dinâmica de parâmetros
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DroneControllerCompleto::onSetParameters, this, std::placeholders::_1));
}
```

### Mecanismo de callback de parâmetros (`onSetParameters`)

Quando um parâmetro é alterado via `ros2 param set`, o método `onSetParameters` é chamado. Ele delega para:

- `apply_enabled_param()` → valida tipo bool e atualiza `enabled_` sob mutex
- `apply_override_active_param()` → valida tipo bool, captura posição atual como hold pose e atualiza `override_active_`

```cpp
// Exemplo: ativar override externo captura a posição atual como "hold"
bool DroneControllerCompleto::apply_override_active_param(
  const rclcpp::Parameter & p,
  rcl_interfaces::msg::SetParametersResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  override_active_ = p.as_bool();
  if (override_active_) {
    // Captura X, Y, Z atuais como posição de hold
    hold_x_ned_ = current_x_ned_;
    hold_y_ned_ = current_y_ned_;
    hold_z_ned_ = current_z_ned_;
    hold_valid_ = true;
  }
  return true;
}
```

## 4. `setup_publishers()`

Cria todos os `rclcpp::Publisher` do nó:

```cpp
void DroneControllerCompleto::setup_publishers()
{
  // Setpoint de posição → MAVROS → PX4
  raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    "/uav1/mavros/setpoint_raw/local", 10);

  // Fim de trajetória (data=true quando todos os WPs foram atingidos)
  trajectory_finished_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/trajectory_finished", 10);

  // Progresso da trajetória [0.0, 100.0]
  progress_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
    "/trajectory_progress", 10);

  // Índice do último waypoint atingido
  waypoint_reached_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "/waypoint_reached", 10);

  // Pose do último waypoint atingido (usada para sanitização XY no próximo takeoff)
  mission_latch_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mission_latch_pose", 10);

  // Waypoint goal atual (status/monitoring — topic configurável)
  waypoint_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    waypoint_goal_status_topic_, 10);

  // Lista de waypoints da trajetória (status/monitoring — topic configurável)
  waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    waypoints_status_topic_, 10);

  // Estado da FSM com QoS transient_local (novos subscribers recebem o último valor)
  state_voo_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "/drone_controller/state_voo",
    rclcpp::QoS(1).transient_local());
}
```

### Tabela de publishers

| Publisher | Tópico | Tipo | QoS | Quando publica |
|-----------|--------|------|-----|----------------|
| `raw_pub_` | `/uav1/mavros/setpoint_raw/local` | `PositionTarget` | 10 | A cada ciclo de 10 ms |
| `trajectory_finished_pub_` | `/trajectory_finished` | `Bool` | 10 | Ao completar trajetória (`data=true`) |
| `progress_publisher_` | `/trajectory_progress` | `Float32` | 10 | Ao completar trajetória (100.0) |
| `waypoint_reached_pub_` | `/waypoint_reached` | `Int32` | 10 | Quando drone atinge cada waypoint |
| `mission_latch_pose_pub_` | `/mission_latch_pose` | `PoseStamped` | 10 | Quando drone atinge waypoint |
| `waypoint_goal_pub_` | `waypoint_goal_status_topic_` | `PoseStamped` | 10 | Heartbeat periódico + mudanças de goal |
| `waypoints_pub_` | `waypoints_status_topic_` | `PoseArray` | 10 | Heartbeat periódico |
| `state_voo_pub_` | `/drone_controller/state_voo` | `Int32` | transient_local(1) | Mudança de estado + heartbeat |

> **`transient_local` em `state_voo_pub_`:** garante que qualquer nó que se inscrever no tópico **após** a publicação receberá imediatamente o último valor conhecido do estado. Útil para dashboards e missões que inicializam depois do controlador.

## 5. `setup_subscribers()`

Cria todos os subscribers e conecta seus callbacks:

```cpp
void DroneControllerCompleto::setup_subscribers()
{
  // Estado do FCU (armed, mode "OFFBOARD"/"MANUAL"/etc.)
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/uav1/mavros/state", 10,
    [this](const mavros_msgs::msg::State::SharedPtr msg) {
      current_state_ = *msg;  // cópia simples (struct pequena)
    });

  // Estado estendido do FCU (landed_state: LANDING, ON_GROUND, etc.)
  extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
    "/uav1/mavros/extended_state", 10,
    [this](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
      last_extended_state_ = *msg;
      last_extended_state_time_ = this->now();  // timestamp para detectar mensagem obsoleta
      extended_state_received_ = true;          // flag de "já recebi ao menos uma vez"
    });

  // Lista de waypoints de trajetória (3D)
  waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    waypoints_cmd_topic_, 1,  // QoS 1: apenas o mais recente importa
    std::bind(&DroneControllerCompleto::waypoints_callback, this, std::placeholders::_1));

  // Waypoint único (takeoff ou manobra em voo)
  waypoint_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    waypoint_goal_cmd_topic_, 1,
    std::bind(&DroneControllerCompleto::waypoint_goal_callback, this, std::placeholders::_1));

  // Odometria local (posição + velocidade + orientação)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/uav1/mavros/local_position/odom", 10,
    std::bind(&DroneControllerCompleto::odometry_callback, this, std::placeholders::_1));

  // Comando de yaw rate externo (rotação no lugar)
  yaw_override_sub_ = this->create_subscription<drone_control::msg::YawOverride>(
    "/uav1/yaw_override/cmd", 10,
    std::bind(&DroneControllerCompleto::yaw_override_callback, this, std::placeholders::_1));

  // Waypoint único 4D (X, Y, Z, yaw)
  waypoint_goal_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4D>(
    "/waypoint_goal_4d", 1,
    std::bind(&DroneControllerCompleto::waypoint_goal_4d_callback, this, std::placeholders::_1));

  // Lista de waypoints 4D (X, Y, Z, yaw por waypoint)
  waypoints_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4DArray>(
    "/waypoints_4d", 1,
    std::bind(&DroneControllerCompleto::waypoints_4d_callback, this, std::placeholders::_1));
}
```

### Tabela de subscribers

| Subscriber | Tópico | Tipo | QoS depth | Callback |
|------------|--------|------|-----------|----------|
| `state_sub_` | `/uav1/mavros/state` | `State` | 10 | lambda inline |
| `extended_state_sub_` | `/uav1/mavros/extended_state` | `ExtendedState` | 10 | lambda inline |
| `waypoints_sub_` | `waypoints_cmd_topic_` | `PoseArray` | 1 | `waypoints_callback` |
| `waypoint_goal_sub_` | `waypoint_goal_cmd_topic_` | `PoseStamped` | 1 | `waypoint_goal_callback` |
| `odom_sub_` | `/uav1/mavros/local_position/odom` | `Odometry` | 10 | `odometry_callback` |
| `yaw_override_sub_` | `/uav1/yaw_override/cmd` | `YawOverride` | 10 | `yaw_override_callback` |
| `waypoint_goal_4d_sub_` | `/waypoint_goal_4d` | `Waypoint4D` | 1 | `waypoint_goal_4d_callback` |
| `waypoints_4d_sub_` | `/waypoints_4d` | `Waypoint4DArray` | 1 | `waypoints_4d_callback` |

> **QoS depth = 1** nos tópicos de waypoints: garante que apenas o comando **mais recente** é processado, descartando mensagens anteriores na fila.

## 6. `setup_services()`

Cria os service clients para MAVROS e inicia o timer de controle:

```cpp
void DroneControllerCompleto::setup_services()
{
  // Client para mudar o modo de voo (ex.: "OFFBOARD")
  mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
    "/uav1/mavros/set_mode");

  // Client para armar/desarmar o drone
  arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
    "/uav1/mavros/cmd/arming");

  // Bloqueia até os serviços MAVROS estarem disponíveis
  // (MAVROS pode demorar alguns segundos para inicializar)
  while (!mode_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/set_mode...");
  }
  while (!arm_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/cmd/arming...");
  }

  // Timer principal: chama control_loop() a cada 10 ms = 100 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&DroneControllerCompleto::control_loop, this));

  // Timer de heartbeat de /waypoint_goal (frequência configurável)
  if (monitor_waypoint_goal_rate_hz_ > 0.0) {
    auto period = std::chrono::nanoseconds(
      static_cast<int64_t>(1e9 / monitor_waypoint_goal_rate_hz_));
    monitor_waypoint_goal_timer_ = this->create_wall_timer(
      period,
      std::bind(&DroneControllerCompleto::monitor_waypoint_goal_heartbeat, this));
  }

  // Timer de heartbeat de /waypoints
  if (monitor_waypoints_rate_hz_ > 0.0) { /* análogo */ }

  // Timer de heartbeat de /drone_controller/state_voo
  if (publish_state_voo_ && state_voo_pub_rate_hz_ > 0.0) { /* análogo */ }
}
```

### Por que bloquear esperando os serviços?

O PX4/MAVROS exige que o controlador envie setpoints **antes** de ser possível solicitar ARM/OFFBOARD. Se o nó inicializar e tentar enviar comandos antes que o MAVROS esteja pronto, as chamadas de serviço falharão silenciosamente. A espera bloqueante garante que quando `control_loop()` começa a executar, os serviços já estão ativos.

## 7. `init_variables()`

Inicializa todas as variáveis de estado da FSM com valores seguros:

```cpp
void DroneControllerCompleto::init_variables()
{
  state_voo_ = 0;              // começa em WAIT
  controlador_ativo_ = false;  // trajetória inativa
  pouso_em_andamento_ = false; // não está pousando

  // Contadores de ciclo e de ativação
  cycle_count_ = 0;
  takeoff_counter_ = 0;
  takeoff_target_z_ = -1.0;   // sentinel: -1.0 = "não inicializado"

  // Flags de sequência OFFBOARD+ARM
  offboard_activated_ = false;
  offboard_mode_confirmed_ = false;
  arm_requested_ = false;
  activation_confirmed_ = false;
  initial_stream_count_ = 0;
  initial_stream_done_ = false;
  post_offboard_stream_count_ = 0;
  post_offboard_stream_done_ = false;

  // Waypoint inicial: origem + hover_altitude
  last_waypoint_goal_.pose.position.x = 0.0;
  last_waypoint_goal_.pose.position.y = 0.0;
  last_waypoint_goal_.pose.position.z = config_.hover_altitude;

  // Trajetória limpa
  trajectory_waypoints_.clear();
  trajectory_started_ = false;
  current_waypoint_idx_ = 0;
  last_waypoint_reached_idx_ = -1;

  // Odometria (atualizada pelo callback a cada mensagem)
  current_z_real_ = 0.0;
  current_x_ned_ = current_y_ned_ = current_z_ned_ = 0.0;
  current_vx_ned_ = current_vy_ned_ = current_vz_ned_ = 0.0;

  // Latch pose (XY do último waypoint atingido para sanitização de takeoff)
  has_latch_pose_ = false;

  // Yaw override
  yaw_override_enabled_ = false;
  yaw_rate_cmd_ = 0.0;
  yaw_override_timeout_s_ = 0.3;

  // Hold setpoint (posição de segurança durante watchdog/override)
  hold_valid_ = false;
  hold_x_ned_ = hold_y_ned_ = hold_z_ned_ = 0.0;

  // Planner codegen
  planner_initialized_ = false;

  // 4D mode flags
  using_4d_goal_ = false;
  trajectory_4d_mode_ = false;
}
```

### Sentinel `takeoff_target_z_ = -1.0`

O valor `-1.0` é usado como sentinela para detectar se `takeoff_target_z_` foi corretamente inicializado antes do inicio da subida. Em `handle_state1_takeoff()`, se o sentinel ainda estiver presente (o que indicaria um bug de sequência), o controlador calcula um valor de fallback seguro:

```cpp
if (takeoff_target_z_ < 0.0) {
  // Fallback de segurança: sobe pelo menos takeoff_z_boost acima da altitude atual
  takeoff_target_z_ = std::max(
    config_.hover_altitude,
    current_z_real_ + config_.takeoff_z_boost);
}
```
