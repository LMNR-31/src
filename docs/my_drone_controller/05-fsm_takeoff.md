# 05 — FSM Estado 1: Decolagem (`fsm_takeoff.cpp`)

O arquivo `src/fsm_takeoff.cpp` implementa toda a lógica do **Estado 1 (TAKEOFF)** da FSM, dividida em três camadas sequenciais que garantem a correta ativação do modo OFFBOARD e do ARM no PX4/MAVROS antes de iniciar a subida.

## 1. Visão Geral da Sequência de Decolagem

O Estado 1 executa uma sequência de **5 etapas**, verificadas a cada ciclo de 10 ms do `control_loop()`:

```
Etapa 1: Streaming pré-ARM
         Publica INITIAL_STREAM_THRESHOLD (≥20) setpoints em posição fixa
         → Define initial_stream_done_ = true
         ↓
Etapa 2: Solicitar modo OFFBOARD
         Chama request_arm_and_offboard_activation() → request_offboard()
         → Define offboard_activated_ = true
         ↓
Etapa 3: Aguardar confirmação de OFFBOARD pelo FCU
         wait_for_offboard_mode(): monitora current_state_.mode == "OFFBOARD"
         → Define offboard_mode_confirmed_ = true
         ↓
Etapa 3.5: Streaming pós-OFFBOARD (1,5 s)
           Publica POST_OFFBOARD_STREAM_THRESHOLD (150) setpoints
           → Define post_offboard_stream_done_ = true
           ↓
Etapa 4: Solicitar ARM
         Chama request_arm() apenas após OFFBOARD confirmado + 1,5 s de stream
         → Define arm_requested_ = true
         ↓
Etapa 5: Subida até takeoff_target_z_
         publish_takeoff_climb_setpoint() + finalize_takeoff_on_altitude_reached()
         → state_voo_ = 2 (HOVER) quando altitude atingida
```

## 2. Por que OFFBOARD e ARM são enviados separadamente?

Enviar ARM **ao mesmo tempo** que OFFBOARD (ou imediatamente depois) faz o PX4 rejeitar o ARM, porque a transição de modo ainda não foi completada internamente no FCU.

```
❌ Errado (ARM rejeitado):
   t=0:  request_offboard()
   t=0:  request_arm()   ← PX4 ignora, modo ainda não é OFFBOARD

✅ Correto (sequência usada neste controlador):
   t=0:    request_offboard()
   t=0..N: aguarda current_state_.mode == "OFFBOARD"
   t=N:    stream_post_offboard_setpoints() por 1,5 s
   t=N+1.5: request_arm()   ← PX4 aceita
```

## 3. `handle_state1_takeoff()` — Dispatcher da Sequência

```cpp
void DroneControllerCompleto::handle_state1_takeoff()
{
  // ── Etapa 1: Streaming pré-ARM ──────────────────────────────────────────
  if (!initial_stream_done_) {
    stream_initial_setpoints();
    return;
  }

  // ── Etapa 2: Solicitar OFFBOARD ─────────────────────────────────────────
  if (!offboard_activated_) {
    request_arm_and_offboard_activation();
    return;
  }

  // ── Etapa 3: Aguardar confirmação de OFFBOARD ───────────────────────────
  if (!offboard_mode_confirmed_) {
    wait_for_offboard_mode();
    return;
  }

  // ── Etapa 3.5: Streaming pós-OFFBOARD (1,5 s) ───────────────────────────
  if (!post_offboard_stream_done_) {
    stream_post_offboard_setpoints();
    return;
  }

  // ── Etapa 4: Solicitar ARM ───────────────────────────────────────────────
  if (!arm_requested_) {
    request_arm();
    arm_requested_ = true;
    return;
  }

  // ── Etapa 5: Aguardar confirmação ARM + subir ────────────────────────────
  if (!activation_confirmed_) {
    if (!wait_for_offboard_arm_confirmation()) { return; }
  }

  // Proteção contra sentinel -1.0 (não deve ocorrer em operação normal)
  if (takeoff_target_z_ < 0.0) {
    takeoff_target_z_ = std::max(
      config_.hover_altitude,
      current_z_real_ + config_.takeoff_z_boost);
  }

  const double target_altitude = takeoff_target_z_;
  publish_takeoff_climb_setpoint(target_altitude);
  finalize_takeoff_on_altitude_reached(target_altitude);
}
```

## 4. `stream_initial_setpoints()` — Streaming Pré-ARM

```cpp
void DroneControllerCompleto::stream_initial_setpoints()
{
  // Publica setpoint fixo na posição de takeoff
  publishPositionTarget(
    last_waypoint_goal_.pose.position.x,
    last_waypoint_goal_.pose.position.y,
    last_waypoint_goal_.pose.position.z,
    0.0, MASK_POS_ONLY);

  initial_stream_count_++;

  if (initial_stream_count_ >= INITIAL_STREAM_THRESHOLD) {
    initial_stream_done_ = true;
    // Próximo ciclo: solicitar OFFBOARD
  }
}
```

> **Por que `last_waypoint_goal_`?** Esse campo foi preenchido em `handle_single_takeoff_waypoint_command()` antes de `state_voo_` ser mudado para 1. Garante que o FCU receba setpoints na posição correta de decolagem.

## 5. `stream_post_offboard_setpoints()` — Streaming Pós-OFFBOARD

```cpp
void DroneControllerCompleto::stream_post_offboard_setpoints()
{
  // Continua publicando o mesmo setpoint fixo
  publishPositionTarget(
    last_waypoint_goal_.pose.position.x,
    last_waypoint_goal_.pose.position.y,
    last_waypoint_goal_.pose.position.z,
    0.0, MASK_POS_ONLY);

  post_offboard_stream_count_++;

  if (post_offboard_stream_count_ >= POST_OFFBOARD_STREAM_THRESHOLD) {
    post_offboard_stream_done_ = true;
    // 150 setpoints × 10 ms = 1,5 s de stream em modo OFFBOARD
    // Próximo ciclo: enviar ARM
  }
}
```

## 6. `wait_for_offboard_mode()` — Aguardar Confirmação de OFFBOARD

```cpp
void DroneControllerCompleto::wait_for_offboard_mode()
{
  if (current_state_.mode == "OFFBOARD") {
    offboard_mode_confirmed_ = true;
    activation_time_ = this->now();  // reinicia timer para ARM timeout
    return;
  }

  // Timeout: se o FCU não confirmou dentro de offboard_confirm_timeout,
  // resetar flags e tentar de novo
  if ((this->now() - activation_time_).seconds() > config_.offboard_confirm_timeout) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ Timeout OFFBOARD (%.0f s)! Tentando novamente...",
      config_.offboard_confirm_timeout);
    offboard_activated_ = false;
    offboard_mode_confirmed_ = false;
    arm_requested_ = false;
    post_offboard_stream_count_ = 0;
    post_offboard_stream_done_ = false;
  }
}
```

### Diagrama de timeout

```
Timeout de offboard_confirm_timeout (padrão: 5 s):
  ┌──────────────────────────────────┐
  │ offboard_activated_ = true       │
  │ activation_time_ = now()         │
  │                                  │
  │  ciclos...                       │
  │                                  │
  │  if mode == "OFFBOARD":          │
  │    offboard_mode_confirmed_=true │← confirma
  │                                  │
  │  else if elapsed > 5 s:          │
  │    reset flags → retry           │← timeout, tenta de novo
  └──────────────────────────────────┘
```

## 7. `wait_for_offboard_arm_confirmation()` — Aguardar ARM

```cpp
bool DroneControllerCompleto::wait_for_offboard_arm_confirmation()
{
  if (current_state_.armed && current_state_.mode == "OFFBOARD") {
    activation_confirmed_ = true;
    takeoff_counter_ = 0;
    // Enfileira comando TAKEOFF para rastreabilidade
    if (!takeoff_cmd_id_) {
      takeoff_cmd_id_ = cmd_queue_.enqueue(CommandType::TAKEOFF, ...);
    }
    return true;
  }

  // Timeout de activation_timeout (padrão: 5 s):
  if ((this->now() - activation_time_).seconds() > config_.activation_timeout) {
    // Reset completo: recomeça da etapa 2 (OFFBOARD request)
    offboard_activated_ = false;
    offboard_mode_confirmed_ = false;
    arm_requested_ = false;
    activation_confirmed_ = false;
    post_offboard_stream_count_ = 0;
    post_offboard_stream_done_ = false;
    return false;
  }
  return false;
}
```

## 8. `takeoff_target_z_` Fixo — Prevenção de Subida Infinita

### O problema (bug clássico)

Se o Z alvo for recalculado a cada ciclo de 10 ms como `current_z_real_ + boost`:
```
ciclo 1: alvo = 0.0 + 0.7 = 0.7 m → drone sobe
ciclo 2: alvo = 0.1 + 0.7 = 0.8 m → alvo sobe junto com o drone
ciclo 3: alvo = 0.2 + 0.7 = 0.9 m → ∞ (nunca alcançado)
```

### A solução

`takeoff_target_z_` é calculado **uma única vez**, no momento em que o comando de takeoff é recebido, em `handle_single_takeoff_waypoint_command()`:

```cpp
// Em handle_single_takeoff_waypoint_command():
const double z_cmd = safe_pose.position.z;
takeoff_target_z_ = std::min(
  config_.max_altitude,
  std::max(config_.min_altitude, z_cmd));
// Exemplo: z_cmd=2.0 → takeoff_target_z_ = clamp(2.0, 0.2, 500.0) = 2.0 m
```

Em `handle_state1_takeoff()`, esse valor fixo é usado:
```cpp
const double target_altitude = takeoff_target_z_;  // valor fixo, não muda
publish_takeoff_climb_setpoint(target_altitude);
finalize_takeoff_on_altitude_reached(target_altitude);
```

## 9. `publish_takeoff_climb_setpoint()` — Publicar Setpoint de Subida

```cpp
void DroneControllerCompleto::publish_takeoff_climb_setpoint(double target_alt)
{
  if (using_4d_goal_) {
    // Modo 4D: publica com yaw absoluto
    publishPositionTargetWithYaw(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      target_alt, goal_yaw_rad_);
  } else {
    // Modo 3D: publica apenas posição
    publishPositionTarget(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      target_alt, 0.0, MASK_POS_ONLY);
  }
  takeoff_counter_++;
}
```

## 10. `finalize_takeoff_on_altitude_reached()` — Detecção de Chegada

```cpp
void DroneControllerCompleto::finalize_takeoff_on_altitude_reached(double target_alt)
{
  // Verifica se chegou à altitude alvo com margem de histerese
  double arrival_threshold = target_alt - config_.hover_altitude_margin;
  if (current_z_real_ < arrival_threshold) { return; }  // ainda subindo

  // Altitude atingida: confirma TAKEOFF na fila de comandos
  if (takeoff_cmd_id_) {
    cmd_queue_.confirm(*takeoff_cmd_id_, true);
    takeoff_cmd_id_.reset();
  }

  // Enfileira HOVER
  hover_cmd_id_ = cmd_queue_.enqueue(CommandType::HOVER, ...);

  // Transita para Estado 2 (HOVER)
  state_voo_ = 2;
  takeoff_counter_ = 0;
}
```

## 11. `sanitize_takeoff_xy()` — Sanitização do XY de Decolagem

Esta função garante que o drone não decole em direção à origem (0,0) caso o comando tenha sido publicado antes que o latch pose estivesse disponível.

```cpp
geometry_msgs::msg::Pose DroneControllerCompleto::sanitize_takeoff_xy(
  const geometry_msgs::msg::Pose & pose)
{
  double xy_dist = std::hypot(pose.position.x, pose.position.y);

  if (xy_dist < config_.takeoff_xy_origin_threshold_m && has_latch_pose_) {
    double age = (this->now() - last_latch_pose_time_).seconds();
    if (age < config_.latch_pose_max_age_s) {
      // Substitui XY pelo latch pose (último waypoint atingido)
      geometry_msgs::msg::Pose sanitized = pose;
      sanitized.position.x = last_latch_pose_.position.x;
      sanitized.position.y = last_latch_pose_.position.y;
      return sanitized;
    }
  }
  return pose;  // XY original está OK
}
```

**Exemplo prático:**
```
Missão anterior: drone atingiu WP em X=5.0, Y=3.0 → latch_pose = (5.0, 3.0)
Nova missão publica: waypoint_goal XY=(0.0, 0.0), Z=2.0
   → xy_dist = 0.0 < threshold(0.2m) && latch válido (age < 10s)
   → sanitize retorna XY=(5.0, 3.0), Z=2.0
   → drone decola de volta para cima do pad, não para a origem
```

## 12. Exemplo completo: sequência de logs durante decolagem

```
[INFO] 📡 [STREAM] Iniciando streaming inicial (meta: 20 mensagens)...
[INFO] ✅ [STREAM] Streaming inicial concluído (20 setpoints). OFFBOARD+ARM...
[INFO] 📡 Solicitando modo OFFBOARD (ARM aguardará confirmação do FCU)...
[INFO] ✅ [ID=1] SET_MODE OFFBOARD aceito pelo FCU
[INFO] ✅ FCU confirmou modo OFFBOARD — aguardando ARM na próxima etapa...
[INFO] 📡 [STREAM] OFFBOARD confirmado — streaming prolongado (150 × 10 ms = 1,5 s)...
[INFO] ✅ [STREAM] Streaming pós-OFFBOARD concluído. Enviando ARM...
[INFO] 🔋 [ID=2] Solicitando ARM...
[INFO] ✅ [ID=2] ARM confirmado pelo FCU
[INFO] ✅ OFFBOARD+ARM CONFIRMADOS! Iniciando decolagem...
[INFO] ⬆️ Decolando para 2.0 metros...
[INFO]    Posição: X=5.00, Y=3.00, Z=2.0
[INFO] 📈 Decolando... Z_alvo=2.0m | Z_real=0.83m | Tempo=1.0s
[INFO] ✅ Decolagem concluída! Altitude = 2.01m
```
