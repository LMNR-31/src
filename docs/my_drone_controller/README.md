# Documentação: `my_drone_controller`

Bem-vindo à documentação do pacote `my_drone_controller`. Esta documentação está organizada em blocos temáticos para facilitar a leitura e a consulta.

## Índice

| Arquivo | Conteúdo |
|---------|----------|
| [00 — Inventário Completo de Arquivos](00-inventario-arquivos-src.md) | Lista completa de todos os 16 arquivos de `my_drone_controller/src` com papel, funções, relação com a FSM e **o que cada bloco de código faz** |
| [01 — Visão Geral](01-visao-geral.md) | Objetivo do nó, arquitetura ROS 2/MAVROS/PX4, módulos, FSM 5 estados e fluxo de dados |
| [02 — Build e Execução](02-build-e-execucao.md) | Dependências, compilação via `colcon`, execução, parâmetros ROS 2 e tópicos cmd vs status |
| [03 — Setup (Parte 1)](03-drone_controller_completo-parte1-setup.md) | `includes`, namespace, construtor, `load_parameters`, `setup_publishers`, `setup_subscribers`, `setup_services`, `init_variables` |
| [04 — Loop e Setpoints (Parte 2)](04-drone_controller_completo-parte2-loop-e-setpoints.md) | Funções `publishPositionTarget*`, máscaras de tipo, watchdog, `publish_hold_setpoint` e `control_loop` |
| [05 — FSM Decolagem](05-fsm_takeoff.md) | `fsm_takeoff.cpp`: streaming pré-ARM, OFFBOARD separado do ARM, confirmação, `takeoff_target_z_` fixo |
| [06 — FSM Hover](06-fsm_hover.md) | `fsm_hover.cpp`: hover 3D/4D, detecção de pouso via `ExtendedState`, transição para estado 3 |
| [07 — FSM Trajetória](07-fsm_trajectory.md) | `fsm_trajectory.cpp`: planner codegen, cálculo de yaw, detecção de waypoint atingido, latch pose, finalização |
| [08 — FSM Pouso](08-fsm_landing.md) | `fsm_landing.cpp`: `complete_landing`, `handle_state4_disarm_reset`, timeout e transição para estado 0 |
| [09 — APIs e Exemplos de Uso](09-apis-e-exemplos-de-uso.md) | Como comandar o controlador via tópicos, exemplos C++ e observabilidade |
| [10 — Simulação e tmux](10-simulacao-e-tmux.md) | Como o `session.yml` do MRS UAV Gazebo Simulator foi modificado para integrar o `drone_node` |
| [11 — Subscribers e Callbacks](11-subscribers-e-callbacks.md) | Tabela completa de subscribers, detalhamento de cada callback, guards anti-echo e exemplos C++ de publicação |
| [12 — `main.cpp` e Estado 0](12-main-e-estado0.md) | Linha a linha: ponto de entrada do executável (`main.cpp`) e estado ocioso da FSM (`fsm_state0_wait.cpp`) |
| [13 — Validação de Waypoints](13-waypoint-validation.md) | Linha a linha: `waypoint_validation.cpp` — verificações de NaN/Inf, limites de altitude e distância XY |
| [14 — Fila de Comandos](14-command-queue.md) | Linha a linha: `command_queue.hpp` + `command_queue.cpp` — rastreabilidade, histórico, timeout e log de auditoria |
| [15 — Planner e Controlador Codegen](15-codegen-planner-e-controlador.md) | Linha a linha: `TrajectoryPlanner_codegen.cpp` (polinômio) + `Drone_codegen.cpp` (PID de posição) |

## Resumo rápido do pacote

```
my_drone_controller/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── drone_config.yaml          ← parâmetros ROS 2 (altitudes, timeouts, etc.)
├── include/
│   ├── drone_config.h             ← struct DroneConfig (valores padrão)
│   ├── TrajectoryPlanner_codegen.h ← planner MATLAB/codegen
│   ├── Drone_codegen.h            ← controlador de posição MATLAB/codegen
│   └── my_drone_controller/
│       ├── drone_controller_completo.hpp  ← declaração completa da classe
│       ├── command_queue.hpp              ← fila de comandos rastreável
│       └── waypoint_validation.hpp        ← validação de waypoints
└── src/
    ├── main.cpp                           ← ponto de entrada
    ├── drone_controller_completo.cpp      ← setup, callbacks, loop principal
    ├── fsm_state0_wait.cpp                ← Estado 0: aguardando
    ├── fsm_takeoff.cpp                    ← Estado 1: decolagem
    ├── fsm_hover.cpp                      ← Estado 2: hover
    ├── fsm_trajectory.cpp                 ← Estado 3: trajetória
    ├── fsm_landing.cpp                    ← Estado 4: pouso
    ├── command_queue.cpp
    ├── waypoint_validation.cpp
    ├── Drone_codegen.cpp                  ← controlador PID (MATLAB codegen)
    ├── TrajectoryPlanner_codegen.cpp      ← planner polinomial (MATLAB codegen)
    ├── cos.cpp                            ← stub vazio (MATLAB codegen)
    ├── minOrMax.cpp                       ← saturação ±12 (MATLAB codegen)
    ├── mldivide.cpp                       ← solver LU (MATLAB codegen)
    ├── rt_nonfinite.cpp                   ← constantes NaN/Inf (MATLAB codegen)
    └── sqrt.cpp                           ← stub vazio (MATLAB codegen)
```

## Convenções adotadas nesta documentação

- Nomes de variáveis membros seguem o sufixo `_` (ex.: `state_voo_`, `config_`).
- Nomes de tópicos ROS 2 são prefixados com `/uav1/` quando específicos ao veículo.
- Exemplos de código C++ são autocontidos e compiláveis com as dependências listadas.
- Linguagem: **português**, estilo acadêmico.
