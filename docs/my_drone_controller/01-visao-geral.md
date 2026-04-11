# 01 — Visão Geral do `my_drone_controller`

## 1. Objetivo do nó `drone_controller_completo`

O nó `drone_controller_completo` implementa um **controlador autônomo de VANT** baseado em ROS 2. Seu papel central é orquestrar toda a sequência de voo — desde o arming, decolagem, navegação por waypoints e pouso — utilizando os serviços e tópicos disponibilizados pelo **MAVROS** para comunicar-se com o firmware **PX4** via MAVLink.

A classe principal é `DroneControllerCompleto`, definida no namespace `drone_control` e declarada em `include/my_drone_controller/drone_controller_completo.hpp`. Ela herda de `rclcpp::Node`, seguindo o padrão de nós gerenciados do ROS 2.

## 2. Visão Geral da Pilha Tecnológica

```
┌──────────────────────────────────────────┐
│          Aplicação / Missão              │
│  (publica /waypoints, /waypoint_goal,    │
│   /waypoints_4d, /waypoint_goal_4d)      │
└────────────────────┬─────────────────────┘
                     │ ROS 2 Topics
┌────────────────────▼─────────────────────┐
│      drone_controller_completo           │
│   (my_drone_controller::drone_node)      │
│                                          │
│  FSM 5 estados + CommandQueue            │
│  + TrajectoryPlanner_codegen             │
│  + Drone_codegen (controlador PID)       │
└────────────────────┬─────────────────────┘
                     │ ROS 2 Services & Topics
┌────────────────────▼─────────────────────┐
│              MAVROS                      │
│  (bridge ROS 2 ↔ MAVLink)               │
│  Tópicos: /uav1/mavros/...              │
└────────────────────┬─────────────────────┘
                     │ MAVLink (serial/UDP)
┌────────────────────▼─────────────────────┐
│             PX4 Autopilot                │
│  (firmware no FCU: Pixhawk, etc.)        │
└──────────────────────────────────────────┘
```

### Componentes principais da pilha

| Componente | Papel |
|------------|-------|
| **ROS 2 (Humble/Rolling)** | Middleware de comunicação entre nós |
| **MAVROS** | Converte mensagens ROS 2 em MAVLink e vice-versa |
| **PX4** | Firmware do controlador de voo; executa os modos ARM/OFFBOARD/LAND |
| **`drone_controller_completo`** | Lógica de missão, FSM, publicação de setpoints |
| **TrajectoryPlanner_codegen** | Planner polinomial gerado por MATLAB Coder |
| **Drone_codegen** | Controlador de posição PID gerado por MATLAB Coder |

## 3. Arquitetura por Módulos e Dependências

O pacote é compilado como um único executável (`drone_node`) composto por múltiplos módulos C++:

```
drone_node
├── main.cpp                      → ponto de entrada, inicializa rclcpp e faz spin()
├── drone_controller_completo.cpp → construtor, setup, callbacks, loop de controle
├── fsm_state0_wait.cpp           → Estado 0: aguarda waypoint
├── fsm_takeoff.cpp               → Estado 1: decolagem (OFFBOARD + ARM + climb)
├── fsm_hover.cpp                 → Estado 2: hover estacionário
├── fsm_trajectory.cpp            → Estado 3: execução de trajetória
├── fsm_landing.cpp               → Estado 4: pouso e DISARM
├── command_queue.cpp             → sistema de rastreabilidade de comandos
├── waypoint_validation.cpp       → validação de NaN/Inf e limites físicos
├── Drone_codegen.cpp             → controlador de posição (codegen MATLAB)
├── TrajectoryPlanner_codegen.cpp → planner de trajetória (codegen MATLAB)
└── rt_nonfinite.cpp              → suporte a floats não-finitos (codegen)
```

### Dependências de headers

| Header | Origem | Papel |
|--------|--------|-------|
| `drone_controller_completo.hpp` | `my_drone_controller` | Declaração da classe principal |
| `drone_config.h` | `my_drone_controller` | Struct `DroneConfig` com todos os parâmetros |
| `command_queue.hpp` | `my_drone_controller` | Fila de comandos auditável |
| `waypoint_validation.hpp` | `my_drone_controller` | Funções `validate_waypoint` / `validate_pose` |
| `TrajectoryPlanner_codegen.h` | `my_drone_controller` | API do planner polinomial |
| `Drone_codegen.h` | `my_drone_controller` | API do controlador PID |
| `rclcpp/rclcpp.hpp` | ROS 2 | Node, Timer, Publisher, Subscriber |
| `mavros_msgs/...` | MAVROS | State, ExtendedState, PositionTarget, SetMode, CommandBool |
| `geometry_msgs/...` | ROS 2 std | PoseStamped, PoseArray |
| `nav_msgs/msg/odometry.hpp` | ROS 2 std | Odometria local |
| `drone_control/msg/...` | pacote `drone_control` | YawOverride, Waypoint4D, Waypoint4DArray |

## 4. Máquina de Estados (FSM) — 5 Estados (0..4)

O controlador implementa uma FSM linear com cinco estados:

```
       ┌─────────┐
       │ Estado 0 │  Aguardando waypoint (idle)
       │  (WAIT)  │
       └────┬─────┘
            │ Recebe /waypoint_goal ou /waypoints (1 wp, Z >= land_z_threshold)
            ▼
       ┌─────────┐
       │ Estado 1 │  Decolagem: streaming → OFFBOARD → ARM → climb
       │(TAKEOFF) │
       └────┬─────┘
            │ current_z_real_ >= takeoff_target_z_ - hover_altitude_margin
            ▼
       ┌─────────┐
       │ Estado 2 │  Hover: aguarda /waypoints (2+ wps) ou /waypoints_4d
       │ (HOVER)  │◄──────────────────────────────────────────────┐
       └────┬─────┘                                               │
            │ controlador_ativo_ == true                          │
            ▼                                                     │
       ┌───────────┐                                              │
       │  Estado 3 │  Trajetória: executa waypoints sequenciais   │
       │(TRAJECTORY)│──(todos waypoints atingidos)────────────────┘
       └────┬──────┘
            │ pouso_em_andamento_ == true  (qualquer estado 1/2/3)
            ▼
       ┌─────────┐
       │ Estado 4 │  Pouso: aguarda timeout → DISARM → volta ao 0
       │(LANDING) │
       └────┬─────┘
            │ !current_state_.armed  (FCU confirma DISARM)
            ▼
       ┌─────────┐
       │ Estado 0 │  Pronto para nova decolagem
       └─────────┘
```

### Descrição de cada estado

| Estado | Valor | Nome | Descrição |
|--------|-------|------|-----------|
| 0 | `0` | WAIT | Nó ligado, aguardando o primeiro comando de waypoint. Log periódico a cada 10 s. |
| 1 | `1` | TAKEOFF | Executa a sequência: streaming pré-ARM → OFFBOARD → ARM → climb até `takeoff_target_z_`. |
| 2 | `2` | HOVER | Drone em altitude de hover, publicando setpoint fixo. Aguarda `/waypoints` para ativar trajetória. |
| 3 | `3` | TRAJECTORY | Executa a lista `trajectory_waypoints_` sequencialmente. Usa `TrajectoryPlanner_codegen` + `Drone_codegen` para setpoints com feedforward de velocidade. |
| 4 | `4` | LANDING | Aguarda `landing_timeout` após detecção de pouso, solicita DISARM e transita para 0 ao receber confirmação do FCU. |

### Transições de estado

| De → Para | Condição de transição |
|-----------|----------------------|
| 0 → 1 | Recepção de waypoint único com Z ≥ `land_z_threshold` (takeoff) |
| 1 → 2 | `current_z_real_` ≥ `takeoff_target_z_` − `hover_altitude_margin` |
| 2 → 3 | `controlador_ativo_` = true (disparado por `/waypoints` com 2+ waypoints) |
| 3 → 2 | Todos os waypoints atingidos (`finalize_trajectory_complete`) |
| 2/3 → 4 | `pouso_em_andamento_` = true (by `trigger_landing`) |
| 4 → 0 | FCU confirma DISARM (`!current_state_.armed`) |
| qualquer → 4 | `autopilot_indicates_landing()` = true (ExtendedState: LANDING ou ON_GROUND) |

## 5. Diagrama Textual do Fluxo de Dados

```
Operador/Missão
   │
   ├── /waypoint_goal       (PoseStamped) ──► waypoint_goal_callback()
   ├── /waypoints           (PoseArray)   ──► waypoints_callback()
   ├── /waypoint_goal_4d    (Waypoint4D)  ──► waypoint_goal_4d_callback()
   ├── /waypoints_4d        (Waypoint4DArray) ──► waypoints_4d_callback()
   └── /uav1/yaw_override/cmd (YawOverride) ──► yaw_override_callback()

MAVROS
   ├── /uav1/mavros/state              ──► current_state_    (armed, mode)
   ├── /uav1/mavros/extended_state     ──► last_extended_state_ (landed_state)
   └── /uav1/mavros/local_position/odom ──► odometry_callback()
                                            (current_x/y/z_ned_, current_yaw_rad_)

drone_controller_completo
   │  control_loop() @ 100 Hz
   │  ├── FSM dispatch (state 0..4)
   │  ├── publishPositionTarget*()  ──► /uav1/mavros/setpoint_raw/local
   │  ├── request_offboard()        ──► /uav1/mavros/set_mode   (service)
   │  └── request_arm/disarm()      ──► /uav1/mavros/cmd/arming (service)
   │
   Outputs (tópicos de observabilidade):
   ├── /drone_controller/state_voo    (Int32, transient_local)
   ├── /trajectory_finished           (Bool)
   ├── /trajectory_progress           (Float32)
   ├── /waypoint_reached              (Int32)
   └── /mission_latch_pose            (PoseStamped)
```
