# 02 — Build e Execução

## 1. Dependências

### 1.1 Dependências de pacotes ROS 2 (`package.xml`)

| Pacote | Tipo | Papel |
|--------|------|-------|
| `ament_cmake` | buildtool | sistema de build CMake do ROS 2 |
| `rclcpp` | build/exec | API C++ do ROS 2 (nós, publishers, subscribers, timers) |
| `geometry_msgs` | build/exec | `PoseStamped`, `PoseArray`, `Pose` |
| `nav_msgs` | build/exec | `Odometry` (posição e velocidade local) |
| `sensor_msgs` | build/exec | mensagens de sensores genéricas |
| `std_msgs` | build/exec | `Bool`, `Float32`, `Int32` |
| `mavros_msgs` | build/exec | `State`, `ExtendedState`, `PositionTarget`, `SetMode`, `CommandBool` |
| `rcl_interfaces` | build/exec | `SetParametersResult` (callback de parâmetros) |
| `drone_control` | build/exec | tipos customizados: `YawOverride`, `Waypoint4D`, `Waypoint4DArray` |

### 1.2 Dependências de headers internos

| Header | Localização | Papel |
|--------|-------------|-------|
| `drone_config.h` | `include/` | struct `DroneConfig` com valores padrão e documentação inline |
| `drone_controller_completo.hpp` | `include/my_drone_controller/` | declaração completa da classe |
| `command_queue.hpp` | `include/my_drone_controller/` | sistema de rastreabilidade de comandos |
| `waypoint_validation.hpp` | `include/my_drone_controller/` | funções de validação de waypoints |
| `TrajectoryPlanner_codegen.h` | `include/` | planner polinomial gerado por MATLAB Coder |
| `Drone_codegen.h` | `include/` | controlador de posição PID gerado por MATLAB Coder |

### 1.3 `CMakeLists.txt` — estrutura relevante

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_drone_controller)

# C++ 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(drone_control REQUIRED)

include_directories(include include/lib src)

# Executável único que agrega todos os módulos
add_executable(drone_node
  src/main.cpp
  src/command_queue.cpp
  src/waypoint_validation.cpp
  src/drone_controller_completo.cpp
  src/fsm_state0_wait.cpp
  src/fsm_takeoff.cpp
  src/fsm_hover.cpp
  src/fsm_trajectory.cpp
  src/fsm_landing.cpp
  src/Drone_codegen.cpp
  src/TrajectoryPlanner_codegen.cpp
  src/rt_nonfinite.cpp
)

ament_target_dependencies(drone_node
  rclcpp nav_msgs geometry_msgs sensor_msgs
  std_msgs mavros_msgs rcl_interfaces drone_control
)

install(TARGETS drone_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

ament_package()
```

> **Nota:** O executável final se chama `drone_node`, porém o nó ROS 2 se registra com o nome `drone_controller_completo` (definido no construtor via `Node("drone_controller_completo")`).

## 2. Como Compilar

### 2.1 Pré-requisitos

- ROS 2 Humble (ou Rolling) instalado e sourced.
- Workspace ROS 2 configurado (ex.: `~/ros2_ws/src/`).
- Pacote `drone_control` (mensagens customizadas) disponível no mesmo workspace.

### 2.2 Compilação com `colcon`

```bash
# Posicione-se na raiz do workspace
cd ~/ros2_ws

# Build apenas do pacote (recomendado para ciclos rápidos)
colcon build --packages-select my_drone_controller

# Ou build de todos os pacotes
colcon build

# Recarregue o ambiente
source install/setup.bash
```

### 2.3 Compilação com símbolos de debug

```bash
colcon build --packages-select my_drone_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## 3. Como Executar o Nó

### 3.1 Execução simples (parâmetros padrão)

```bash
ros2 run my_drone_controller drone_node
```

### 3.2 Execução com arquivo de parâmetros

```bash
ros2 run my_drone_controller drone_node \
  --ros-args \
  --params-file /path/to/drone_config.yaml
```

O arquivo `drone_config.yaml` é instalado em:

```
share/my_drone_controller/config/drone_config.yaml
```

Portanto, após `source install/setup.bash`:

```bash
ros2 run my_drone_controller drone_node \
  --ros-args \
  --params-file $(ros2 pkg prefix my_drone_controller)/share/my_drone_controller/config/drone_config.yaml
```

### 3.3 Execução via launch file (exemplo)

```python
# launch/drone_controller.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_drone_controller'),
        'config', 'drone_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='my_drone_controller',
            executable='drone_node',
            name='drone_controller_completo',
            parameters=[config],
            output='screen',
        )
    ])
```

## 4. Parâmetros ROS 2 declarados em `load_parameters()`

Todos os parâmetros são declarados e lidos na função `load_parameters()`, chamada no construtor do nó. Os valores padrão estão definidos na struct `DroneConfig` em `drone_config.h`.

### 4.1 Parâmetros de altitude

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|--------|-----------|
| `hover_altitude` | `double` | `2.0` | Altitude de hover após decolagem [m] |
| `hover_altitude_margin` | `double` | `0.05` | Margem de histerese para detectar chegada à altitude [m] |
| `max_altitude` | `double` | `500.0` | Altitude máxima permitida para waypoints recebidos [m] |
| `min_altitude` | `double` | `0.2` | Altitude mínima de voo [m] |
| `land_z_threshold` | `double` | `0.1` | Z abaixo do qual o drone é considerado pousado [m] |

### 4.2 Parâmetros de waypoint e trajetória

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|--------|-----------|
| `waypoint_duration` | `double` | `4.0` | Tempo de permanência em cada waypoint durante trajetória [s] |
| `max_waypoint_distance` | `double` | `100.0` | Distância máxima permitida para |X| ou |Y| de waypoints [m] |

### 4.3 Parâmetros de decolagem e sanitização XY

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|--------|-----------|
| `takeoff_z_boost` | `double` | `0.7` | Incremento mínimo de Z acima da odometria atual após ARM [m] |
| `takeoff_xy_origin_threshold_m` | `double` | `0.2` | Raio em torno da origem dentro do qual o XY de takeoff é substituído pelo latch pose [m] |
| `latch_pose_max_age_s` | `double` | `10.0` | Tempo máximo para um latch pose ser considerado válido [s] |

### 4.4 Timeouts

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|--------|-----------|
| `activation_timeout` | `double` | `5.0` | Tempo máximo aguardando OFFBOARD+ARM do FCU [s] |
| `command_timeout` | `double` | `15.0` | Tempo antes de um comando pendente ser marcado como TIMEOUT [s] |
| `landing_timeout` | `double` | `3.0` | Tempo aguardando após detecção de pouso antes de DISARM [s] |
| `offboard_confirm_timeout` | `double` | `5.0` | Tempo aguardando FCU confirmar modo OFFBOARD antes de retentar [s] |

### 4.5 Parâmetros de controle e override

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|--------|-----------|
| `enabled` | `bool` | `true` | Liga/desliga a publicação de setpoints (FSM é pausada quando `false`) |
| `override_active` | `bool` | `false` | Quando `true`, congela a FSM e publica hold setpoint da posição atual |

### 4.6 Parâmetros de monitoramento

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|--------|-----------|
| `monitor_waypoint_goal_rate_hz` | `double` | `5.0` | Frequência do heartbeat de `/waypoint_goal` [Hz] |
| `monitor_waypoints_rate_hz` | `double` | `1.0` | Frequência do heartbeat de `/waypoints` [Hz] |
| `monitor_publish_only_when_active` | `bool` | `true` | Suprime heartbeats quando não há missão ativa |
| `publish_state_voo` | `bool` | `true` | Habilita publicação periódica de `/drone_controller/state_voo` |
| `state_voo_pub_rate_hz` | `double` | `1.0` | Frequência do heartbeat de estado [Hz] |

### 4.7 Parâmetros de tópicos cmd vs status

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|--------|-----------|
| `waypoints_cmd_topic` | `string` | `/waypoints` | Tópico de entrada de waypoints |
| `waypoints_status_topic` | `string` | `/waypoints` | Tópico de saída de waypoints (monitoramento) |
| `waypoint_goal_cmd_topic` | `string` | `/waypoint_goal` | Tópico de entrada do waypoint goal |
| `waypoint_goal_status_topic` | `string` | `/waypoint_goal` | Tópico de saída do waypoint goal (monitoramento) |

> **Atenção:** Quando `cmd_topic == status_topic` (padrão), o nó emite um aviso de que o mesmo tópico é usado para entrada e saída, podendo causar conflitos de publishers. Recomenda-se separar em ambientes de produção, por exemplo:
> ```yaml
> waypoints_status_topic: "/waypoints_status"
> waypoint_goal_status_topic: "/waypoint_goal_status"
> ```

### 4.8 Alteração de parâmetros em tempo de execução

Os parâmetros `enabled` e `override_active` podem ser alterados dinamicamente via `ros2 param set`:

```bash
# Pausar o controlador
ros2 param set /drone_controller_completo enabled false

# Reativar
ros2 param set /drone_controller_completo enabled true

# Ativar override externo (FSM congelada, drone mantém posição atual)
ros2 param set /drone_controller_completo override_active true
```

O callback `onSetParameters` valida o tipo do parâmetro e aplica a mudança atomicamente sob `mutex_`.

## 5. Observações sobre tópicos cmd vs status

A separação entre tópicos de **comando** (entrada) e **status** (saída/monitoramento) é uma característica importante deste controlador:

- **Tópicos cmd** (`waypoints_cmd_topic`, `waypoint_goal_cmd_topic`): recebem comandos de outros nós (ex.: `pad_waypoint_supervisor`, `mission_manager`).
- **Tópicos status** (`waypoints_status_topic`, `waypoint_goal_status_topic`): publicados pelo controlador como heartbeat periódico para monitoramento (ex.: visualização no RViz, registro de missão).

Quando ambos apontam para o mesmo tópico (comportamento padrão), o nó implementa um contador de auto-eco (`skip_self_waypoints_count_`, `skip_self_waypoint_goal_count_`) para evitar que suas próprias publicações sejam reprocessadas como novos comandos.
