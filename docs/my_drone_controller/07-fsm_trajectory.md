# 07 — FSM Estado 3: Trajetória (`fsm_trajectory.cpp`)

O arquivo `src/fsm_trajectory.cpp` implementa o **Estado 3 (TRAJECTORY)** da FSM. Este estado é responsável por executar a lista de waypoints armazenada em `trajectory_waypoints_`, usando o **`TrajectoryPlanner_codegen`** como planner polinomial e o **`Drone_codegen`** como controlador de posição com feedforward de velocidade.

## 1. Funções implementadas

| Função | Descrição |
|--------|-----------|
| `detect_and_handle_landing_in_trajectory()` | Detecta e gerencia pouso detectado durante trajetória |
| `initialize_trajectory()` | Inicializa planner + controlador na primeira iteração |
| `compute_yaw_for_trajectory_waypoint()` | Calcula yaw "look-at" para o waypoint atual |
| `publish_trajectory_waypoint_setpoint()` | Publica setpoint de posição+yaw para o waypoint atual |
| `log_trajectory_progress()` | Log periódico de progresso |
| `finalize_trajectory_complete()` | Conclui trajetória, publica eventos, volta a HOVER |
| `handle_state3_trajectory()` | Handler principal do Estado 3, chamado a 100 Hz |

## 2. `handle_state3_trajectory()` — Handler Principal

```cpp
void DroneControllerCompleto::handle_state3_trajectory()
{
  // 1. Verificar pouso durante trajetória
  if (detect_and_handle_landing_in_trajectory()) { return; }

  // 2. Detectar pouso por flag interna (fallback sem ExtendedState)
  if (pouso_em_andamento_ && !controlador_ativo_) {
    state_voo_ = 4;
    return;
  }

  // 3. Inicializar trajetória (apenas na 1ª iteração)
  if (!initialize_trajectory()) { return; }

  // 4. Verificação de bounds do índice
  if (current_waypoint_idx_ < 0 ||
      static_cast<size_t>(current_waypoint_idx_) >= trajectory_waypoints_.size()) {
    state_voo_ = 2;  // erro: volta ao hover
    return;
  }

  // 5. Publicar setpoint
  if (planner_initialized_) {
    // Modo planner: posição + velocidade feedforward + yaw
    double elapsed = (this->now() - trajectory_start_time_).seconds();
    double Xd[3], Vd[3], Ad[3];
    planner_.getNextSetpoint(elapsed, Xd, Vd, Ad);

    // Alimentar odometria no controlador PID
    drone_ctrl_.r[0] = current_x_ned_;
    drone_ctrl_.r[1] = current_y_ned_;
    drone_ctrl_.r[2] = current_z_ned_;
    drone_ctrl_.dr[0] = current_vx_ned_;
    drone_ctrl_.dr[1] = current_vy_ned_;
    drone_ctrl_.dr[2] = current_vz_ned_;
    drone_ctrl_.PositionCtrl(Xd, Vd, Ad);

    double yaw_follow = compute_yaw_for_trajectory_waypoint(
      current_waypoint_idx_, at_last_wp);

    publishPositionTargetWithVelocityAndYaw(
      Xd[0], Xd[1], Xd[2],
      Vd[0], Vd[1], drone_ctrl_.zdot_des,
      yaw_follow);
  } else {
    // Fallback: 1 único waypoint → setpoint de posição simples
    publish_trajectory_waypoint_setpoint(current_waypoint_idx_);
  }

  log_trajectory_progress(current_waypoint_idx_);

  // 6. Detectar chegada ao waypoint atual
  // ... (ver seção 6)
}
```

## 3. `initialize_trajectory()` — Inicialização do Planner

Chamada apenas quando `trajectory_started_ = false` (primeira iteração do Estado 3):

```cpp
bool DroneControllerCompleto::initialize_trajectory()
{
  if (trajectory_started_) { return true; }

  if (trajectory_waypoints_.empty()) {
    state_voo_ = 2;  // sem waypoints: volta ao hover
    return false;
  }

  trajectory_start_time_ = this->now();
  trajectory_started_ = true;
  current_waypoint_idx_ = 0;
  planner_initialized_ = false;

  int n = static_cast<int>(trajectory_waypoints_.size()) - 1;
  if (n >= 1) {
    // Construir vetor flat de waypoints: [X0..Xn, Y0..Yn, Z0..Zn]
    planner_.waypoints.resize(3 * (n + 1));
    for (int i = 0; i <= n; ++i) {
      planner_.waypoints[i]               = trajectory_waypoints_[i].position.x;  // X
      planner_.waypoints[(n+1) + i]       = trajectory_waypoints_[i].position.y;  // Y
      planner_.waypoints[2*(n+1) + i]     = trajectory_waypoints_[i].position.z;  // Z
    }
    planner_.segmentTimes.assign(static_cast<size_t>(n), waypoint_duration_);
    planner_.numSegments = n;
    planner_.init();   // computa splines polinomiais

    drone_ctrl_.init();  // reseta estado do controlador PID
    planner_initialized_ = true;
  }
  // n < 1 → waypoint único → fallback para setpoint simples

  return true;
}
```

### Formato do vetor `planner_.waypoints`

Para N+1 waypoints, o vetor tem tamanho 3×(N+1) com layout:

```
[ X_0, X_1, ..., X_N,   (N+1 valores de X)
  Y_0, Y_1, ..., Y_N,   (N+1 valores de Y)
  Z_0, Z_1, ..., Z_N ]  (N+1 valores de Z)
```

**Exemplo com 3 waypoints:**
```cpp
// WP0=(1,0,2), WP1=(3,2,2), WP2=(5,0,2)
planner_.waypoints = {1.0, 3.0, 5.0,   // X
                      0.0, 2.0, 0.0,   // Y
                      2.0, 2.0, 2.0};  // Z
planner_.segmentTimes = {4.0, 4.0};    // 4 s por segmento
planner_.numSegments = 2;
planner_.init();  // gera splines WP0→WP1 e WP1→WP2
```

## 4. `compute_yaw_for_trajectory_waypoint()` — Cálculo de Yaw

```cpp
double DroneControllerCompleto::compute_yaw_for_trajectory_waypoint(
  int idx, bool at_last_wp)
{
  const auto & wp = trajectory_waypoints_[idx];
  double yaw_follow = 0.0;

  if (at_last_wp) {
    // No último waypoint: congela o yaw calculado (evita jitter por imprecisão de posição)
    if (!at_last_waypoint_yaw_fixed_) {
      double dx = wp.position.x - current_x_ned_;
      double dy = wp.position.y - current_y_ned_;
      yaw_follow = std::atan2(dy, dx);
      final_waypoint_yaw_ = yaw_follow;
      at_last_waypoint_yaw_fixed_ = true;
    } else {
      yaw_follow = final_waypoint_yaw_;  // usa valor congelado
    }
  } else {
    // Waypoints intermediários: recalcula a cada ciclo
    double dx = wp.position.x - current_x_ned_;
    double dy = wp.position.y - current_y_ned_;
    yaw_follow = std::atan2(dy, dx);
    final_waypoint_yaw_ = yaw_follow;
    at_last_waypoint_yaw_fixed_ = false;
  }

  return yaw_follow;
}
```

**Exemplo de cálculo:**
```
Drone em (2.0, 1.0), próximo WP em (4.0, 3.0):
  dx = 4.0 - 2.0 = 2.0
  dy = 3.0 - 1.0 = 2.0
  yaw = atan2(2.0, 2.0) = 0.785 rad (≈ 45° Nordeste)
```

### Por que congelar o yaw no último waypoint?

Quando o drone está muito próximo do último waypoint, os erros de posição `(dx, dy)` ficam muito pequenos e o `atan2` fica instável (pequenas perturbações geram grandes variações de yaw). Congelar o yaw calculado na chegada ao último waypoint evita esse comportamento.

## 5. Modo Planner vs. Modo Fallback (Waypoint Único)

| Condição | Modo | Setpoint publicado |
|----------|------|-------------------|
| `planner_initialized_ = true` (≥2 WPs) | **Planner** | Posição + velocidade feedforward (Vd) + zdot_des do PID + yaw |
| `planner_initialized_ = false` (1 WP) | **Fallback** | Apenas posição + yaw (sem velocidade) |

### Vantagem do modo planner

O `TrajectoryPlanner_codegen` gera uma trajetória **polinomial contínua** entre os waypoints, evitando paradas bruscas. O `Drone_codegen` (controlador PID) usa a odometria atual e a referência do planner para calcular `zdot_des`, corrigindo erros de altitude de forma independente do planner.

## 6. Detecção de Waypoint Atingido

A cada ciclo, verifica se o drone atingiu o waypoint atual com tolerância:

```cpp
constexpr double XY_TOL = 0.10;  // tolerância radial XY [m]
constexpr double Z_TOL  = 0.15;  // tolerância absoluta Z [m]

const auto & wp = trajectory_waypoints_[current_waypoint_idx_];
double dx = wp.position.x - current_x_ned_;
double dy = wp.position.y - current_y_ned_;
double dz = std::abs(wp.position.z - current_z_real_);
double dist_xy = std::sqrt(dx*dx + dy*dy);

if (dist_xy <= XY_TOL && dz <= Z_TOL &&
    current_waypoint_idx_ != last_waypoint_reached_idx_)
{
  // Publica /waypoint_reached com o índice atingido
  waypoint_reached_pub_->publish(reached_msg);

  // Publica /mission_latch_pose (XY de referência para próximo takeoff)
  mission_latch_pose_pub_->publish(latch_msg);
  last_latch_pose_      = latch_msg.pose;
  has_latch_pose_       = true;

  // Avança para o próximo waypoint
  current_waypoint_idx_++;

  // Verifica se a trajetória terminou
  if (current_waypoint_idx_ >= static_cast<int>(trajectory_waypoints_.size())) {
    finalize_trajectory_complete();
    state_voo_ = 2;  // volta ao HOVER
    return;
  }
}
```

## 7. `finalize_trajectory_complete()` — Conclusão da Trajetória

```cpp
void DroneControllerCompleto::finalize_trajectory_complete()
{
  // Confirma TRAJECTORY na fila de comandos
  if (trajectory_cmd_id_) {
    cmd_queue_.confirm(*trajectory_cmd_id_, true);
    trajectory_cmd_id_.reset();
  }

  // Publica /trajectory_finished = true
  std_msgs::msg::Bool done_msg;
  done_msg.data = true;
  trajectory_finished_pub_->publish(done_msg);

  // Publica /trajectory_progress = 100.0
  std_msgs::msg::Float32 progress_msg;
  progress_msg.data = 100.0;
  progress_publisher_->publish(progress_msg);

  // Atualiza posição de hover para o último waypoint atingido
  if (!trajectory_waypoints_.empty()) {
    last_waypoint_goal_.header.stamp = this->now();
    last_waypoint_goal_.header.frame_id = "map";
    last_waypoint_goal_.pose = trajectory_waypoints_.back();
  }
  // Próxima chamada a handle_state2_hover() mantém drone na posição final
}
```

> **Importante:** `/trajectory_finished` é publicado **apenas com `data=true`** na conclusão. Não há publicação com `data=false` no início da trajetória.

## 8. Latch Pose: Referência para o Próximo Takeoff

Cada vez que o drone atinge um waypoint, a pose exata daquele waypoint é publicada em `/mission_latch_pose` e armazenada localmente em `last_latch_pose_`. Isso serve como referência XY para `sanitize_takeoff_xy()` no próximo ciclo de decolagem.

```
Trajetória: WP0(1,0,2) → WP1(3,2,2) → WP2(5,0,2)

WP0 atingido → latch_pose = (1.0, 0.0, 2.0)
WP1 atingido → latch_pose = (3.0, 2.0, 2.0)
WP2 atingido → latch_pose = (5.0, 0.0, 2.0)  ← último

Próximo takeoff recebe XY=(0,0):
  sanitize_takeoff_xy() detecta XY próximo da origem
  substitui por latch_pose XY = (5.0, 0.0)
  drone decola para cima do último waypoint, não para a origem
```

## 9. Exemplos de Trajetória

### Exemplo 3D — 3 waypoints em altitude constante

```cpp
// Publicar no tópico /waypoints
geometry_msgs::msg::PoseArray msg;
msg.header.frame_id = "map";
msg.header.stamp = this->now();

geometry_msgs::msg::Pose wp0, wp1, wp2;
wp0.position.x = 1.0; wp0.position.y = 0.0; wp0.position.z = 2.0;
wp1.position.x = 3.0; wp1.position.y = 2.0; wp1.position.z = 2.0;
wp2.position.x = 5.0; wp2.position.y = 0.0; wp2.position.z = 2.0;
msg.poses = {wp0, wp1, wp2};

waypoints_publisher->publish(msg);
```

### Exemplo 4D — 2 waypoints com yaw específico por waypoint

```cpp
// Publicar no tópico /waypoints_4d
drone_control::msg::Waypoint4DArray msg;
msg.header.frame_id = "map";
msg.header.stamp = this->now();

drone_control::msg::Waypoint4D wp0, wp1;
wp0.pose.position.x = 2.0; wp0.pose.position.y = 0.0;
wp0.pose.position.z = 2.0; wp0.yaw = 0.0f;     // olhando para Leste
wp1.pose.position.x = 4.0; wp1.pose.position.y = 2.0;
wp1.pose.position.z = 3.0; wp1.yaw = 1.5708f;  // olhando para Norte (π/2 rad)
msg.waypoints = {wp0, wp1};

waypoints_4d_publisher->publish(msg);
```
