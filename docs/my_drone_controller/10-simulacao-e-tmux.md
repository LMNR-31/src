# 10 — Simulação com MRS UAV Gazebo Simulator e tmux (`session.yml`)

Este documento explica como o arquivo `mrs_uav_gazebo_simulator/tmux/one_drone/session.yml` foi modificado para integrar o `my_drone_controller` ao simulador, substituindo parcialmente a pilha padrão MRS UAS por uma configuração customizada de controle direto via OFFBOARD/PX4.

## 1. Contexto: O que é o `session.yml`?

O arquivo `session.yml` é um script de sessão do **tmuxinator**, uma ferramenta que inicia automaticamente múltiplas janelas e painéis do `tmux` com comandos pré-definidos. Ao executar `./start.sh`, todas as janelas da simulação são abertas simultaneamente, com os processos necessários já em execução.

### Como iniciar a simulação

```bash
cd ~/ros2_ws/src/mrs_uav_gazebo_simulator/tmux/one_drone
./start.sh
```

O `start.sh` chama:
```bash
tmuxinator start -p ./session.yml
tmux -L mrs a -t simulation   # ou attach automático
```

## 2. Comparação: Versão Original vs. Versão Customizada

### Versão original (`one_drone_lidar/session.yml` — referência MRS UAS padrão)

```
Windows padrão MRS UAS:
  zenoh_router  → rmw_zenohd
  gazebo        → simulation.launch.py + spawn drone
  px4_api       → mrs_uav_px4_api (API entre MRS core e PX4)
  core          → mrs_uav_core.launch.py (stack completa MRS)
  status        → mrs_uav_status
  autostart     → mrs_uav_autostart
  takeoff       → takeoff.sh
  rviz          → rviz2
  layout        → layout_manager.sh
```

### Versão customizada (`one_drone/session.yml` — este projeto)

```
Windows customizados:
  zenoh_router    → rmw_zenohd                    (igual)
  gazebo          → simulation.launch.py + spawn  (arena customizada)
  px4             → PX4 SITL direto (sem mrs_uav_px4_api)
  tf_fixes        → TF broadcaster + camera static + odom_tf_broadcaster
  status          → drone_status_monitor.py (monitor customizado)
  drone_controller → drone_node (my_drone_controller)
  Terminal YOLO   → yolo_pad_pose_ros2 (detecção de pad de pouso)
  debug_console   → terminal livre
  terminal1..7    → terminais auxiliares
```

**Diferença central:** A pilha MRS (`mrs_uav_core`, `mrs_uav_px4_api`, `mrs_uav_autostart`) foi **substituída** pelo controle direto OFFBOARD via `drone_node`, eliminando a necessidade do framework completo MRS para este caso de uso.

## 3. Análise Detalhada do `session.yml`

### 3.1 Variáveis de Ambiente (`pre_window`)

```yaml
pre_window:
  - export UAV_NAME=uav1
  - export RUN_TYPE=simulation
  - export UAV_TYPE=f450   # modelo de drone: frame F450
  - export PLATFORM_CONFIG=/home/lmnr31/ros2_ws/src/mrs_uav_gazebo_simulator/config/mrs_uav_system/f450.yaml
  - export RMW_IMPLEMENTATION=rmw_zenoh_cpp   # middleware ROS 2: Zenoh
  - export USE_SIM_TIME="true"               # usa relógio do simulador
  - export GZ_SIM_MODEL_PATH=$GZ_SIM_MODEL_PATH:$HOME/ros2_ws/src/arena_2025_desenho:/home/lmnr31/ros2_ws/src/mrs_uav_gazebo_simulator/models/mrs_robot_description/sdf/drones
  - export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/drone_ws/src/arena_2025_desenho
  - export DRONE_CONFIG=$(readlink -f ../../../my_drone_controller/config/drone_config.yaml)
```

#### Variáveis importantes

| Variável | Valor | Propósito |
|----------|-------|-----------|
| `UAV_TYPE` | `f450` | Define o modelo físico do drone usado no Gazebo |
| `RMW_IMPLEMENTATION` | `rmw_zenoh_cpp` | Middleware de comunicação ROS 2 (Zenoh em vez de DDS) |
| `USE_SIM_TIME` | `"true"` | Todos os nós usam o relógio do simulador (`/clock`) |
| `GZ_SIM_MODEL_PATH` | caminho customizado | Adiciona modelos 3D da arena e do drone ao Gazebo |
| `DRONE_CONFIG` | `readlink -f ../../../my_drone_controller/config/drone_config.yaml` | Caminho absoluto para o arquivo de parâmetros do controlador |

> **`readlink -f`** resolve o caminho relativo `../../../my_drone_controller/config/drone_config.yaml` para um caminho absoluto, garantindo que o controlador encontre o arquivo independentemente do diretório de trabalho.

### 3.2 Window `zenoh_router` — Middleware ROS 2

```yaml
- zenoh_router:
    panes:
      - ros2 run rmw_zenoh_cpp rmw_zenohd
```

Inicia o **roteador Zenoh**, necessário quando `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. O Zenoh é usado em vez do DDS padrão para comunicação mais eficiente em ambientes com múltiplos processos.

> **Por que Zenoh?** O Zenoh oferece melhor performance em ambientes com muitos tópicos ROS 2 e múltiplos nós, além de suporte nativo a comunicação entre máquinas distintas.

### 3.3 Window `gazebo` — Simulador e Spawn do Drone

```yaml
- gazebo:
    panes:
      # Inicia Gazebo com o mundo customizado da arena
      - ros2 launch mrs_uav_gazebo_simulator simulation.launch.py \
          world_file:=/home/lmnr31/ros2_ws/src/arena_2025_desenho/my_arena.world \
          gz_headless:=false gz_verbose:=true

      # Aguarda Gazebo inicializar e chama o serviço de spawn do drone
      - |
        waitForGazebo; sleep 5
        ros2 service call /mrs_drone_spawner/spawn mrs_msgs/srv/String \
          "{value: '1 --f450 --enable-realsense-down --enable-realsense-front \
            --enable-rangefinder --pos -0.85 0.70 1 1.57'}"
```

#### Parâmetros de spawn do drone

| Parâmetro | Valor | Significado |
|-----------|-------|-------------|
| `1` | ID do drone | Cria `uav1` |
| `--f450` | modelo | Frame F450 (quadrotor) |
| `--enable-realsense-down` | sensor | Câmera RealSense apontada para baixo (detecção de pad) |
| `--enable-realsense-front` | sensor | Câmera RealSense frontal |
| `--enable-rangefinder` | sensor | Sensor de distância (altímetro) |
| `--pos -0.85 0.70 1 1.57` | posição | X=-0.85, Y=0.70, Z=1.0m, yaw=1.57 rad |

> **`waitForGazebo`** é um script auxiliar do MRS UAS que bloqueia até o serviço `/mrs_drone_spawner/spawn` estar disponível, evitando que o spawn falhe se o Gazebo ainda não inicializou completamente.

### 3.4 Window `px4` — PX4 SITL Direto

```yaml
- px4:
    panes:
      - |
        pkill -f "/opt/ros/jazzy/lib/px4/px4" || true
        rm -rf /tmp/sitl_uav_1

        export PX4_SIM_MODEL=f450
        export PX4_SIMULATOR=gz
        export PX4_GZ_MODEL_NAME=uav1
        export PX4_GZ_WORLD=default
        export CONNECT_TO_QGC=1

        /opt/ros/jazzy/lib/px4/px4 \
          ~/ros2_ws/install/mrs_uav_gazebo_simulator/share/mrs_uav_gazebo_simulator/ROMFS/px4fmu_common \
          -s etc/init.d-posix/rcS -i 1 -w /tmp/sitl_uav_1
```

#### O que cada linha faz

```bash
pkill -f "/opt/ros/jazzy/lib/px4/px4" || true
# Mata instância anterior do PX4 se existir (reinicialização limpa)
# "|| true" garante que o script não falhe se não houver processo

rm -rf /tmp/sitl_uav_1
# Limpa diretório de estado anterior do PX4 SITL

export PX4_SIM_MODEL=f450      # modelo físico para o SITL
export PX4_SIMULATOR=gz        # usa Gazebo como simulador (em vez de jMAVSim)
export PX4_GZ_MODEL_NAME=uav1  # nome do modelo no Gazebo
export PX4_GZ_WORLD=default    # mundo padrão do Gazebo
export CONNECT_TO_QGC=1        # habilita conexão com QGroundControl (porta UDP)

/opt/ros/jazzy/lib/px4/px4 \
  ~/ros2_ws/install/.../ROMFS/px4fmu_common \
  # diretório com arquivos de configuração do firmware PX4 SITL
  -s etc/init.d-posix/rcS     # script de inicialização PX4 para POSIX/Linux
  -i 1                        # instância 1 (uav1)
  -w /tmp/sitl_uav_1          # diretório de trabalho para logs e parâmetros
```

> **Por que PX4 direto?** A versão original usa `mrs_uav_px4_api`, que é um intermediário do stack MRS. Nesta versão customizada, o PX4 SITL é iniciado diretamente, e o `drone_node` comunica-se com ele via MAVROS sem camadas intermediárias, simplificando o sistema.

### 3.5 Window `tf_fixes` — Transformações de Referência

```yaml
- tf_fixes:
    panes:
      # TF de fallback para o corpo do drone (body frame)
      - |
        waitForGazebo; sleep 5
        source ~/ros2_ws/install/setup.bash
        ros2 launch drone_control tf_body_fallback.launch.py

      # TF estático da câmera (pose da câmera em relação ao corpo do drone)
      - |
        waitForGazebo; sleep 5
        source ~/ros2_ws/install/setup.bash
        ros2 launch drone_control tf_camera_static.launch.py

      # Broadcaster de TF a partir da odometria (publica uav1/odom → uav1/base_link)
      - |
        waitForGazebo; sleep 5
        source ~/ros2_ws/install/setup.bash
        ros2 run yolo_pad_pose odom_tf_broadcaster
```

#### Por que são necessários?

| Launch/Node | Propósito |
|-------------|-----------|
| `tf_body_fallback.launch.py` | Publica a TF `uav1/base_link` quando o MRS core não está rodando |
| `tf_camera_static.launch.py` | TF estático entre `uav1/base_link` e o frame da câmera RealSense |
| `odom_tf_broadcaster` | Publica TF dinâmica `uav1/odom → uav1/base_link` a partir da odometria MAVROS |

### 3.6 Window `status` — Monitor de Status Customizado

```yaml
- status:
    panes:
      - |
        waitForPX4; sleep 3
        source ~/ros2_ws/install/setup.bash
        python3 ./custom_status/drone_status_monitor.py
```

Inicia o **`DroneStatusMonitor`** (`custom_status/drone_status_monitor.py`), um nó Python que exibe em tempo real um painel curses com:

- CPU e RAM (gráficos ASCII)
- Altitude real vs. altitude alvo
- Estado OFFBOARD + ARM
- Velocidade linear e total
- Bateria (voltagem, corrente, percentual)
- Frequência do loop e uptime
- Até 3 alertas críticos

O monitor subscreve:
- `/uav1/mavros/local_position/pose` (altitude)
- `/uav1/mavros/local_position/velocity_local` (velocidade)
- `/uav1/mavros/state` (armed, mode)
- `/uav1/mavros/battery` (bateria)

### 3.7 Window `drone_controller` — Nó Principal de Controle

```yaml
- drone_controller:
    panes:
      - |
        waitForGazebo; sleep 5
        waitForPX4; sleep 2
        source ~/ros2_ws/install/setup.bash
        ros2 run my_drone_controller drone_node \
          --ros-args --params-file $DRONE_CONFIG \
          2>&1 | tee -a /tmp/drone_logs/drone_controller_$(date +%Y%m%d_%H%M%S).log
```

#### O que cada parte faz

```bash
waitForGazebo; sleep 5   # aguarda Gazebo + 5 s extra para estabilização
waitForPX4; sleep 2      # aguarda PX4 SITL + 2 s extra para MAVROS conectar

source ~/ros2_ws/install/setup.bash
# Carrega o ambiente ROS 2 com todos os pacotes compilados

ros2 run my_drone_controller drone_node \
  --ros-args --params-file $DRONE_CONFIG
# Inicia o drone_node com os parâmetros definidos em drone_config.yaml
# $DRONE_CONFIG foi definido no pre_window como o caminho absoluto do yaml

2>&1 | tee -a /tmp/drone_logs/drone_controller_$(date +%Y%m%d_%H%M%S).log
# Redireciona stdout e stderr para um arquivo de log com timestamp
# tee -a: escreve no arquivo E exibe no terminal
# Arquivo: /tmp/drone_logs/drone_controller_20240115_102300.log
```

#### Comentário sobre tópicos separados (cmd vs status)

```yaml
# Comentário preservado no session.yml para referência futura:
# To avoid publisher conflicts with pad_waypoint_supervisor set:
#   waypoints_cmd_topic:=/waypoints_cmd      (supervisor publishes here)
#   waypoints_status_topic:=/waypoints_status (controller echo goes here)
# Example: ros2 run my_drone_controller drone_node --ros-args \
#   --params-file $DRONE_CONFIG \
#   -p waypoints_cmd_topic:=/waypoints_cmd \
#   -p waypoints_status_topic:=/waypoints_status
```

Quando o `pad_waypoint_supervisor` está ativo, ele publica em `/waypoints_cmd` e o controlador deve subscrever apenas esse tópico, evitando conflito com o tópico de status.

### 3.8 Window `Terminal YOLO` — Detecção de Pad de Pouso

```yaml
- Terminal YOLO:
    panes:
      - |
        waitForGazebo; sleep 5
        source /opt/ros/jazzy/setup.bash
        source ~/ros2_ws/install/setup.bash
        source /home/lmnr31/venvs/yolo/bin/activate
        python3 -m yolo_pad_pose.yolo_pad_pose_ros2 --ros-args \
          -p model_path:=/home/lmnr31/ros2_ws/src/models/landing_pad_yolo/best.pt \
          -p conf:=0.7 \
          -p max_base_range_m:=4.0 \
          -p max_h_range_m:=4.0
```

#### O que este painel faz

Inicia o nó `yolo_pad_pose_ros2`, responsável por detectar o **pad de pouso** usando YOLO v8 na câmera RealSense de baixo.

| Parâmetro | Valor | Significado |
|-----------|-------|-------------|
| `model_path` | `best.pt` | Modelo YOLO treinado para detecção do landing pad |
| `conf` | `0.7` | Limiar de confiança mínima da detecção (70%) |
| `max_base_range_m` | `4.0` | Alcance máximo para câmera frontal [m] |
| `max_h_range_m` | `4.0` | Alcance máximo para câmera de baixo [m] |

O nó publica:
- `/landing_pad/base_relative_position` (`PointStamped`) — posição do pad em relação à câmera frontal
- `/landing_pad/h_relative_position` (`PointStamped`) — posição do pad em relação à câmera de baixo

```bash
# Ativa o venv Python com YOLO instalado
source /home/lmnr31/venvs/yolo/bin/activate
```

## 4. Diagrama Completo da Sessão

```
tmuxinator session.yml
│
├── zenoh_router  → rmw_zenohd (middleware ROS 2)
│
├── gazebo ──────→ Gazebo sim (my_arena.world)
│              └─→ Spawn uav1 (F450 + câmeras + rangefinder)
│
├── px4 ─────────→ PX4 SITL (uav1, gz plugin)
│              └─→ tf_body_fallback.launch.py
│              └─→ tf_camera_static.launch.py
│              └─→ odom_tf_broadcaster
│
├── status ──────→ drone_status_monitor.py (painel curses 2 Hz)
│
├── drone_controller → drone_node --params-file drone_config.yaml
│                      (logs em /tmp/drone_logs/)
│
├── Terminal YOLO → yolo_pad_pose_ros2 (detecção YOLO do pad)
│
├── debug_console → terminal bash livre
│
└── terminal1..7  → terminais auxiliares
```

## 5. Fluxo de Inicialização

```
1. start.sh → tmuxinator inicia todos os painéis

2. zenoh_router inicia rmw_zenohd (middleware)

3. gazebo inicia Gazebo + MRS simulation.launch.py
   Aguarda /mrs_drone_spawner/spawn disponível
   Spawneia uav1 na posição (-0.85, 0.70, 1.0, yaw=1.57)

4. px4 inicia PX4 SITL (conecta ao Gazebo via plugin gz)
   tf_fixes iniciam os broadcasters de TF

5. status inicia drone_status_monitor.py
   (aguarda waitForPX4 + 3 s)

6. drone_controller inicia drone_node
   (aguarda waitForGazebo + 5 s, waitForPX4 + 2 s)
   Lê drone_config.yaml via $DRONE_CONFIG
   Aguarda /uav1/mavros/set_mode e /uav1/mavros/cmd/arming

7. Terminal YOLO inicia yolo_pad_pose_ros2
   (aguarda waitForGazebo + 5 s)

8. Sistema pronto: drone_node em estado 0 (WAIT)
   Aguardando /waypoints ou /waypoint_goal para decolar
```

## 6. Como Enviar o Primeiro Takeoff

Após a inicialização completa:

```bash
# Em qualquer terminal (ex.: debug_console):
source ~/ros2_ws/install/setup.bash

# Verificar que drone_controller está rodando:
ros2 topic echo /drone_controller/state_voo --once
# Deve retornar: data: 0

# Enviar takeoff para X=0, Y=0, Z=2.5m:
ros2 topic pub --once /waypoints geometry_msgs/msg/PoseArray \
  '{header: {frame_id: "map"}, poses: [{position: {x: 0.0, y: 0.0, z: 2.5}}]}'

# Monitorar o progresso:
ros2 topic echo /drone_controller/state_voo
# 0 → 1 (takeoff) → 2 (hover)
```

## 7. Solução de Problemas Comuns

### `waitForPX4` trava

O script `waitForPX4` aguarda o serviço `/uav1/mavros/set_mode` estar disponível. Se o PX4 não inicializar, verificar:

```bash
# No painel px4: verificar se o processo está rodando
ps aux | grep px4

# Verificar se o Gazebo está publicando o plugin gz:
ros2 topic list | grep mavros
```

### `drone_node` trava em "Aguardando /uav1/mavros/set_mode"

MAVROS pode demorar para conectar ao PX4. O `setup_services()` bloqueia até o serviço estar disponível. Verificar:

```bash
# Confirmar que MAVROS está rodando:
ros2 node list | grep mavros

# Ver estado atual:
ros2 topic echo /uav1/mavros/state --once
```

### Logs do drone_controller

Os logs ficam em `/tmp/drone_logs/`. Para ver o log mais recente:

```bash
ls -lt /tmp/drone_logs/ | head -5
tail -100 /tmp/drone_logs/drone_controller_YYYYMMDD_HHMMSS.log
```
