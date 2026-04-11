# 09 — APIs e Exemplos de Uso

Este documento descreve todos os tópicos de entrada aceitos pelo `drone_controller_completo`, seus tipos de mensagem, semântica e exemplos de código C++ para publicadores externos.

## 1. Tópicos de Comando (Entrada)

### 1.1 `/waypoint_goal` — Waypoint único 3D

**Tipo:** `geometry_msgs/msg/PoseStamped`
**QoS:** depth 1

Envia um único waypoint 3D para o controlador. O comportamento depende do estado atual da FSM:

| Estado FSM | Comportamento |
|------------|---------------|
| 0 (WAIT) | Inicia decolagem (se Z ≥ `land_z_threshold`) |
| 2 (HOVER) | Atualiza posição de hover |
| 3 (TRAJECTORY) | Atualiza setpoint de trajetória |
| 1/4 | Ignorado |

**Campos usados:** `pose.position.x`, `pose.position.y`, `pose.position.z`

**Exemplo C++:**
```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class TakeoffSender : public rclcpp::Node {
public:
  TakeoffSender() : Node("takeoff_sender") {
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/waypoint_goal", 1);

    // Aguarda 1s e envia o comando de takeoff
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = 0.0;  // permanecer no X atual
        msg.pose.position.y = 0.0;  // permanecer no Y atual
        msg.pose.position.z = 2.5;  // decolar para 2.5 m
        pub_->publish(msg);
        timer_->cancel();
      });
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffSender>());
  rclcpp::shutdown();
}
```

### 1.2 `/waypoints` — Lista de waypoints 3D

**Tipo:** `geometry_msgs/msg/PoseArray`
**QoS:** depth 1

Envia uma lista de waypoints. O comportamento depende da quantidade de poses e dos valores de Z:

| Condição | Comportamento |
|----------|---------------|
| 1 pose, Z < `land_z_threshold` | Inicia pouso (Estado → 4) |
| 1 pose, Z ≥ `land_z_threshold`, Estado 0 | Inicia decolagem (Estado → 1) |
| ≥2 poses, Estado 2 | Ativa trajetória imediatamente (Estado → 3) |
| ≥2 poses, Estado 0/1 | Armazena para ativar quando entrar no Estado 2 |

**Exemplo C++ — enviar trajetória com 3 waypoints:**
```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

auto make_pose(double x, double y, double z) {
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = 1.0;  // quaternion identidade
  return p;
}

// Em algum callback ou função:
auto pub = node->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 1);

geometry_msgs::msg::PoseArray msg;
msg.header.stamp    = node->now();
msg.header.frame_id = "map";
msg.poses = {
  make_pose(1.0, 0.0, 2.0),  // WP0
  make_pose(3.0, 2.0, 2.0),  // WP1
  make_pose(5.0, 0.0, 2.0),  // WP2
};
pub->publish(msg);
```

**Exemplo — comando de pouso (Z = 0.0 m < land_z_threshold):**
```cpp
geometry_msgs::msg::PoseArray land_msg;
land_msg.header.stamp    = node->now();
land_msg.header.frame_id = "map";
land_msg.poses = { make_pose(0.0, 0.0, 0.0) };  // Z=0 → pouso
pub->publish(land_msg);
```

### 1.3 `/waypoint_goal_4d` — Waypoint único 4D

**Tipo:** `drone_control/msg/Waypoint4D`
**QoS:** depth 1

Waypoint com X, Y, Z e **yaw absoluto** em radianos. Usado para controle preciso de heading durante hover e decolagem.

**Campos da mensagem `Waypoint4D`:**
```
geometry_msgs/Pose pose    # posição X, Y, Z
float32 yaw                # yaw em radianos (NaN = usar yaw atual)
```

**Exemplo C++:**
```cpp
#include "drone_control/msg/waypoint4_d.hpp"

auto pub4d = node->create_publisher<drone_control::msg::Waypoint4D>(
  "/waypoint_goal_4d", 1);

drone_control::msg::Waypoint4D msg;
msg.pose.position.x = 3.0;
msg.pose.position.y = 2.0;
msg.pose.position.z = 2.5;
msg.yaw = 1.5708f;  // 90° = Norte (π/2 rad)
pub4d->publish(msg);
```

**Nota sobre `yaw = NaN`:** Se `msg.yaw = std::numeric_limits<float>::quiet_NaN()`, o controlador usa o yaw atual do drone (`current_yaw_rad_`), sem alterar a orientação.

### 1.4 `/waypoints_4d` — Lista de waypoints 4D

**Tipo:** `drone_control/msg/Waypoint4DArray`
**QoS:** depth 1

Lista de waypoints com yaw por waypoint. Segue a mesma lógica de `/waypoints`, mas com campo `yaw` por ponto.

**Campos da mensagem `Waypoint4DArray`:**
```
std_msgs/Header header
drone_control/Waypoint4D[] waypoints  # array de Waypoint4D
```

**Exemplo C++ — trajetória 4D com 2 waypoints:**
```cpp
#include "drone_control/msg/waypoint4_d_array.hpp"

auto pub4d_arr = node->create_publisher<drone_control::msg::Waypoint4DArray>(
  "/waypoints_4d", 1);

drone_control::msg::Waypoint4DArray msg;
msg.header.stamp    = node->now();
msg.header.frame_id = "map";

drone_control::msg::Waypoint4D wp0, wp1;
wp0.pose.position.x = 2.0;
wp0.pose.position.y = 0.0;
wp0.pose.position.z = 2.0;
wp0.yaw = 0.0f;           // Leste

wp1.pose.position.x = 4.0;
wp1.pose.position.y = 3.0;
wp1.pose.position.z = 3.0;
wp1.yaw = 1.5708f;        // Norte (π/2 rad)

msg.waypoints = {wp0, wp1};
pub4d_arr->publish(msg);
```

### 1.5 `/uav1/yaw_override/cmd` — Yaw Rate Override

**Tipo:** `drone_control/msg/YawOverride`
**QoS:** depth 10

Congela a posição do drone e aplica uma rotação em torno do eixo Z a uma taxa específica.

**Campos da mensagem `YawOverride`:**
```
bool  enable    # true = ativar override, false = desativar
float yaw_rate  # taxa de rotação em rad/s
float timeout   # tempo máximo sem novo comando antes de desativar [s]
```

**Exemplo C++ — girar 0.5 rad/s por 3 segundos:**
```cpp
#include "drone_control/msg/yaw_override.hpp"

auto yaw_pub = node->create_publisher<drone_control::msg::YawOverride>(
  "/uav1/yaw_override/cmd", 10);

// Ativar
drone_control::msg::YawOverride on_msg;
on_msg.enable   = true;
on_msg.yaw_rate = 0.5f;   // rad/s (positivo = anti-horário em NED)
on_msg.timeout  = 0.5f;   // desativa 0,5 s após o último comando
yaw_pub->publish(on_msg);

// Desativar (enviar enable=false ou simplesmente parar de publicar após timeout)
drone_control::msg::YawOverride off_msg;
off_msg.enable   = false;
off_msg.yaw_rate = 0.0f;
yaw_pub->publish(off_msg);
```

## 2. Tópicos de Observabilidade (Saída)

### 2.1 `/drone_controller/state_voo` — Estado da FSM

**Tipo:** `std_msgs/msg/Int32`
**QoS:** `transient_local(1)` (novos subscribers recebem o último valor)

| Valor | Estado | Significado |
|-------|--------|-------------|
| 0 | WAIT | Aguardando waypoint |
| 1 | TAKEOFF | Executando decolagem |
| 2 | HOVER | Em hover, aguardando trajetória |
| 3 | TRAJECTORY | Executando trajetória |
| 4 | LANDING | Pousando e aguardando DISARM |

**Exemplo — monitorar estado:**
```cpp
#include "std_msgs/msg/int32.hpp"

node->create_subscription<std_msgs::msg::Int32>(
  "/drone_controller/state_voo",
  rclcpp::QoS(1).transient_local(),
  [](const std_msgs::msg::Int32::SharedPtr msg) {
    std::cout << "Estado FSM: " << msg->data << std::endl;
  });
```

### 2.2 `/trajectory_finished` — Trajetória Concluída

**Tipo:** `std_msgs/msg/Bool`

Publicado com `data=true` apenas quando **todos os waypoints** da trajetória foram atingidos. **Não é publicado com `data=false`** ao iniciar a trajetória.

```cpp
node->create_subscription<std_msgs::msg::Bool>(
  "/trajectory_finished", 10,
  [](const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      // Trajetória completa → pode publicar nova missão ou pousar
    }
  });
```

### 2.3 `/trajectory_progress` — Progresso da Trajetória

**Tipo:** `std_msgs/msg/Float32`

Valor em `[0.0, 100.0]`. Publicado quando a trajetória é concluída (100.0) via `finalize_trajectory_complete()`.

### 2.4 `/waypoint_reached` — Índice do Waypoint Atingido

**Tipo:** `std_msgs/msg/Int32`

Publicado sempre que o drone entra na zona de tolerância de um waypoint (XY ≤ 10 cm, Z ≤ 15 cm). O valor é o índice (base 0) do waypoint atingido.

```cpp
// Monitorar chegada a cada waypoint:
node->create_subscription<std_msgs::msg::Int32>(
  "/waypoint_reached", 10,
  [](const std_msgs::msg::Int32::SharedPtr msg) {
    std::cout << "Waypoint " << msg->data << " atingido!" << std::endl;
  });
```

### 2.5 `/mission_latch_pose` — Pose do Último Waypoint Atingido

**Tipo:** `geometry_msgs/msg/PoseStamped`

Publicado em cada waypoint atingido durante a trajetória. Usado por outros nós (ex.: `pad_waypoint_supervisor`) como referência de posição XY para o próximo ciclo de takeoff.

### 2.6 `/uav1/mavros/setpoint_raw/local` — Setpoints Enviados ao FCU

**Tipo:** `mavros_msgs/msg/PositionTarget`

Tópico de saída para o MAVROS/PX4. Publicado a 100 Hz pelo controlador. Pode ser monitorado para depuração.

## 3. Tabela Resumo de Tópicos

### Entradas

| Tópico | Tipo | Uso típico |
|--------|------|-----------|
| `/waypoint_goal` | `PoseStamped` | Takeoff único, hover manual |
| `/waypoints` | `PoseArray` | Trajetória 3D ou pouso |
| `/waypoint_goal_4d` | `Waypoint4D` | Takeoff com yaw específico |
| `/waypoints_4d` | `Waypoint4DArray` | Trajetória 4D com heading |
| `/uav1/yaw_override/cmd` | `YawOverride` | Rotação no lugar |

### Saídas

| Tópico | Tipo | Frequência |
|--------|------|-----------|
| `/drone_controller/state_voo` | `Int32` | Na mudança + heartbeat (`state_voo_pub_rate_hz`) |
| `/trajectory_finished` | `Bool` | Apenas ao concluir trajetória |
| `/trajectory_progress` | `Float32` | Apenas ao concluir trajetória (100.0) |
| `/waypoint_reached` | `Int32` | Por waypoint atingido |
| `/mission_latch_pose` | `PoseStamped` | Por waypoint atingido |
| `/waypoint_goal` (status) | `PoseStamped` | Heartbeat (`monitor_waypoint_goal_rate_hz`) |
| `/waypoints` (status) | `PoseArray` | Heartbeat (`monitor_waypoints_rate_hz`) |
| `/uav1/mavros/setpoint_raw/local` | `PositionTarget` | 100 Hz |

## 4. Cenários de Uso Típicos

### Cenário 1: Missão completa takeoff → trajetória → pouso

```
1. Publicar /waypoints com 1 pose (Z=2.0) → takeoff
   Aguardar /drone_controller/state_voo = 2

2. Publicar /waypoints com 3 poses (trajetória)
   Aguardar /trajectory_finished (data=true)

3. Publicar /waypoints com 1 pose (Z=0.0) → pouso
   Aguardar /drone_controller/state_voo = 0
```

### Cenário 2: Takoff com heading fixo (4D)

```
1. Publicar /waypoints_4d com 1 wp (X=2, Y=1, Z=2.5, yaw=0.0)
   → drone decola e mantém heading para Leste

2. Publicar /waypoints_4d com 2+ wps → trajetória com heading por WP
```

### Cenário 3: Rotação de inspeção no hover

```
1. Drone em hover (Estado 2)
2. Publicar /uav1/yaw_override/cmd (enable=true, yaw_rate=0.3, timeout=0.5)
   → publicar a cada 200 ms para manter ativo
3. Publicar /uav1/yaw_override/cmd (enable=false) para parar
```

### Cenário 4: Pausar controlador temporariamente

```bash
# Via linha de comando (enquanto o nó está rodando):
ros2 param set /drone_controller_completo enabled false
# Drone mantém posição atual com setpoints de hold (watchdog 20 Hz)

ros2 param set /drone_controller_completo enabled true
# FSM retoma do ponto onde parou
```

## 5. Validação de Waypoints

Todos os waypoints recebidos são validados pela função `validate_waypoint()` antes de serem processados:

```cpp
// Rejeita se:
// 1. Qualquer coordenada for NaN ou Inf
// 2. |X| > max_waypoint_distance (padrão: 100 m)
// 3. |Y| > max_waypoint_distance (padrão: 100 m)
// 4. Z > max_altitude (padrão: 500 m)
// 5. Z < 0 (altitude negativa)

bool validate_waypoint(const geometry_msgs::msg::PoseStamped & msg,
                       const DroneConfig & config);
```

Waypoints inválidos são descartados com um log de `WARN`:
```
[WARN] ❌ waypoint_goal inválido (NaN/Inf ou fora dos limites físicos) - ignorado
```
