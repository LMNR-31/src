# 04 — `drone_controller_completo.cpp` — Parte 2: Loop e Setpoints

Este documento detalha a segunda metade do arquivo `src/drone_controller_completo.cpp`, cobrindo as funções de **publicação de setpoints**, as **máscaras `type_mask`**, o **watchdog**, `publish_hold_setpoint` e o **loop de controle principal** `control_loop()`.

## 1. Constantes de Máscara (`type_mask`)

A mensagem `mavros_msgs::msg::PositionTarget` usa um campo `type_mask` de 16 bits para indicar ao PX4 quais campos devem ser **ignorados**. Um bit = 1 significa "ignore este campo".

```cpp
// Definidas como constantes estáticas na classe (drone_controller_completo.hpp)

// Bits individuais de "ignore"
static constexpr uint16_t IGNORE_VX       = (1 << 3);   // bit 3
static constexpr uint16_t IGNORE_VY       = (1 << 4);   // bit 4
static constexpr uint16_t IGNORE_VZ       = (1 << 5);   // bit 5
static constexpr uint16_t IGNORE_AFX      = (1 << 6);   // aceleração X
static constexpr uint16_t IGNORE_AFY      = (1 << 7);   // aceleração Y
static constexpr uint16_t IGNORE_AFZ      = (1 << 8);   // aceleração Z
static constexpr uint16_t IGNORE_YAW      = (1 << 10);  // yaw absoluto
static constexpr uint16_t IGNORE_YAW_RATE = (1 << 11);  // yaw rate
```

### Máscaras compostas

| Constante | Valor binário (bits ignorados) | Uso |
|-----------|-------------------------------|-----|
| `MASK_POS_YAWRATE` | VX+VY+VZ+AFX+AFY+AFZ+YAW | Posição (X,Y,Z) + yaw_rate |
| `MASK_POS_YAW` | VX+VY+VZ+AFX+AFY+AFZ+YAW_RATE | Posição (X,Y,Z) + yaw absoluto |
| `MASK_POS_ONLY` | VX+VY+VZ+AFX+AFY+AFZ+YAW+YAW_RATE | Apenas posição (X,Y,Z) |
| `MASK_POS_VEL_YAW` | AFX+AFY+AFZ+YAW_RATE | Posição + velocidade (feedforward) + yaw |

```cpp
// Posição + yaw_rate (ignora vel, acel, yaw absoluto)
static constexpr uint16_t MASK_POS_YAWRATE =
  IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW;

// Posição + yaw absoluto (ignora vel, acel, yaw_rate)
static constexpr uint16_t MASK_POS_YAW =
  IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE;

// Apenas posição (ignora tudo exceto X, Y, Z)
static constexpr uint16_t MASK_POS_ONLY = MASK_POS_YAWRATE | IGNORE_YAW_RATE;

// Posição + velocidade + yaw (ignora só aceleração e yaw_rate)
static constexpr uint16_t MASK_POS_VEL_YAW =
  IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE;
```

## 2. Constantes do Watchdog

```cpp
/// Frequência mínima garantida de publicação de setpoints
static constexpr double MIN_SETPOINT_RATE_HZ = 20.0;

/// Silêncio máximo permitido entre publicações (1/20 Hz = 50 ms)
static constexpr double MAX_SETPOINT_SILENCE_S = 1.0 / MIN_SETPOINT_RATE_HZ;
```

O PX4 **descarta o modo OFFBOARD** se não receber setpoints por mais de ~500 ms. O watchdog garante que, mesmo quando a FSM está pausada (`enabled=false`) ou o override está ativo, setpoints são publicados a pelo menos 20 Hz.

## 3. Constantes de Streaming Pré-ARM

```cpp
/// Mínimo de setpoints antes de solicitar ARM+OFFBOARD (≈200 ms a 100 Hz)
static constexpr int INITIAL_STREAM_THRESHOLD = 20;

/// Setpoints após confirmação de OFFBOARD e ANTES de ARM (150 × 10 ms = 1,5 s)
static constexpr int POST_OFFBOARD_STREAM_THRESHOLD = 150;
```

## 4. Funções de Publicação de Setpoint

### 4.1 `publishPositionTarget()` — posição + yaw_rate

```cpp
void DroneControllerCompleto::publishPositionTarget(
  double x, double y, double z,
  double yaw_rate, uint16_t type_mask)
{
  mavros_msgs::msg::PositionTarget pt;
  pt.header.stamp    = this->now();
  pt.header.frame_id = "map";
  pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  pt.type_mask = type_mask;          // define quais campos o PX4 deve usar
  pt.position.x = static_cast<float>(x);
  pt.position.y = static_cast<float>(y);
  pt.position.z = static_cast<float>(z);
  pt.yaw_rate   = static_cast<float>(yaw_rate);
  raw_pub_->publish(pt);

  // Watchdog: registra instante da última publicação real
  last_setpoint_pub_time_ = pt.header.stamp;
  setpoint_pub_time_initialized_ = true;
}
```

**Quando usar:** setpoints de hover, takeoff, pouso — quando não é necessário controlar o yaw absoluto.

**Exemplo — hover simples em X=2.0, Y=1.0, Z=2.5:**
```cpp
publishPositionTarget(2.0, 1.0, 2.5, 0.0, MASK_POS_ONLY);
```

### 4.2 `publishPositionTargetWithYaw()` — posição + yaw absoluto

```cpp
void DroneControllerCompleto::publishPositionTargetWithYaw(
  double x, double y, double z, double yaw_rad)
{
  mavros_msgs::msg::PositionTarget pt;
  pt.header.stamp    = this->now();
  pt.header.frame_id = "map";
  pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  pt.type_mask = MASK_POS_YAW;  // usa posição + yaw absoluto
  pt.position.x = static_cast<float>(x);
  pt.position.y = static_cast<float>(y);
  pt.position.z = static_cast<float>(z);
  pt.yaw = static_cast<float>(yaw_rad);
  raw_pub_->publish(pt);
  last_setpoint_pub_time_ = pt.header.stamp;
  setpoint_pub_time_initialized_ = true;
}
```

**Quando usar:** hovering 4D, decolagem com yaw específico, trajetória com heading para o próximo waypoint.

**Exemplo — drone em X=3.0, Y=0.0, Z=2.0 olhando para Leste (yaw=0 rad):**
```cpp
publishPositionTargetWithYaw(3.0, 0.0, 2.0, 0.0);
```

### 4.3 `publishPositionTargetWithVelocityAndYaw()` — posição + velocidade feedforward + yaw

```cpp
void DroneControllerCompleto::publishPositionTargetWithVelocityAndYaw(
  double x,  double y,  double z,
  double vx, double vy, double vz,
  double yaw_rad)
{
  mavros_msgs::msg::PositionTarget pt;
  pt.header.stamp    = this->now();
  pt.header.frame_id = "map";
  pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  pt.type_mask = MASK_POS_VEL_YAW;  // usa pos + vel + yaw (sem aceleração)
  pt.position.x  = static_cast<float>(x);
  pt.position.y  = static_cast<float>(y);
  pt.position.z  = static_cast<float>(z);
  pt.velocity.x  = static_cast<float>(vx);   // feedforward XY do planner
  pt.velocity.y  = static_cast<float>(vy);
  pt.velocity.z  = static_cast<float>(vz);   // zdot_des do controlador PID
  pt.yaw         = static_cast<float>(yaw_rad);
  raw_pub_->publish(pt);
  last_setpoint_pub_time_ = pt.header.stamp;
  setpoint_pub_time_initialized_ = true;
}
```

**Quando usar:** trajetória com `TrajectoryPlanner_codegen` + `Drone_codegen`. O planner fornece `(Xd, Vd)` e o controlador PID fornece `zdot_des`.

**Exemplo — seguindo planner com feedforward:**
```cpp
double Xd[3], Vd[3], Ad[3];
planner_.getNextSetpoint(elapsed, Xd, Vd, Ad);
drone_ctrl_.PositionCtrl(Xd, Vd, Ad);

publishPositionTargetWithVelocityAndYaw(
  Xd[0], Xd[1], Xd[2],          // posição desejada
  Vd[0], Vd[1],                  // velocidade feedforward XY
  drone_ctrl_.zdot_des,          // velocidade Z do PID
  yaw_follow);                   // yaw para o próximo waypoint
```

## 5. Watchdog e `publish_hold_setpoint()`

### 5.1 `publish_hold_setpoint()`

Publica um setpoint de posição "congelada" usando a posição capturada em `hold_x/y/z_ned_`. Se o hold não foi capturado ainda, usa a odometria atual.

```cpp
void DroneControllerCompleto::publish_hold_setpoint()
{
  if (hold_valid_) {
    // Usa a posição capturada no momento em que o override/watchdog foi ativado
    publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, 0.0, MASK_POS_ONLY);
  } else {
    // Fallback: usa odometria atual (o drone mantém onde está)
    publishPositionTarget(current_x_ned_, current_y_ned_, current_z_ned_, 0.0, MASK_POS_ONLY);
  }
}
```

### 5.2 Quando o watchdog é ativado

No `control_loop()`, quando `enabled_ = false`:

```cpp
if (!enabled_) {
  // Mantém stream ≥ 20 Hz para não perder OFFBOARD mode
  if ((now - last_setpoint_pub_time_).seconds() >= MAX_SETPOINT_SILENCE_S) {
    publish_hold_setpoint();
  }
  return;  // pula toda a lógica da FSM
}
```

## 6. `control_loop()` — Loop de Controle Principal (100 Hz)

Este é o coração do controlador, chamado pelo `timer_` a cada 10 ms.

```cpp
void DroneControllerCompleto::control_loop()
{
  std::lock_guard<std::mutex> lock(mutex_);  // proteção thread-safe
  cycle_count_++;

  const auto now = this->now();

  // Inicializa watchdog na primeira iteração
  if (!setpoint_pub_time_initialized_) {
    last_setpoint_pub_time_ = now;
    setpoint_pub_time_initialized_ = true;
  }

  // ── 1. Verificação: controlador habilitado? ──────────────────────────────
  if (!enabled_) {
    // FSM pausada: publica hold setpoint para manter OFFBOARD ativo
    if ((now - last_setpoint_pub_time_).seconds() >= MAX_SETPOINT_SILENCE_S) {
      publish_hold_setpoint();
    }
    return;
  }

  // ── 2. Watchdog: timeout de yaw override ────────────────────────────────
  if (yaw_override_enabled_) {
    double since_last = (this->now() - last_yaw_cmd_time_).seconds();
    if (since_last > yaw_override_timeout_s_) {
      // Sem novo comando por mais de yaw_override_timeout_s_:
      // desativa override automaticamente
      yaw_override_enabled_ = false;
      yaw_rate_cmd_ = 0.0;
    }
  }

  // ── 3. Yaw override ativo: congela FSM, publica com yaw_rate ────────────
  if (yaw_override_enabled_ && hold_valid_) {
    publishPositionTarget(
      hold_x_ned_, hold_y_ned_, hold_z_ned_,
      yaw_rate_cmd_, MASK_POS_YAWRATE);
    return;
  }

  // ── 4. Override externo ativo (override_active=true): FSM congelada ─────
  if (override_active_) {
    if (hold_valid_) {
      publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, 0.0, MASK_POS_ONLY);
    } else {
      publishPositionTarget(current_x_ned_, current_y_ned_, current_z_ned_, 0.0, MASK_POS_ONLY);
    }
    return;
  }

  // ── 5. Verificação periódica de timeouts (~10 s) ─────────────────────────
  if (cycle_count_ % 1000 == 0) {
    // Verifica comandos pendentes sem confirmação
    auto timed_out = cmd_queue_.check_timeouts(config_.command_timeout);
    for (auto id : timed_out) {
      RCLCPP_WARN(this->get_logger(), "⏰ [ID=%lu] Comando TIMEOUT!", id);
    }
    // Salva histórico de comandos no arquivo de log
    cmd_queue_.save_log("/tmp/drone_commands.log");
  }

  // ── 6. Despacho para o handler do estado atual ───────────────────────────
  switch (state_voo_) {
    case 0: handle_state0_wait_waypoint(); break;
    case 1: handle_state1_takeoff();       break;
    case 2: handle_state2_hover();         break;
    case 3: handle_state3_trajectory();    break;
    case 4: handle_state4_landing();       break;
    default:
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "❌ Estado FSM inválido: %d", state_voo_);
      break;
  }

  // ── 7. Publica mudança de estado imediatamente ───────────────────────────
  publish_state_voo_on_change();
}
```

### Prioridade de execução no `control_loop()`

```
Checagem de enabled_ (pausa total)
  │
  ▼ (enabled=true)
Watchdog: yaw_override timeout
  │
  ▼
Yaw override ativo? → publica hold+yaw_rate e RETORNA
  │
  ▼ (sem yaw override)
Override externo ativo? → publica hold e RETORNA
  │
  ▼ (sem override)
Verificação periódica de timeouts (a cada 1000 ciclos = ~10 s)
  │
  ▼
Despacho FSM: state 0..4
  │
  ▼
publish_state_voo_on_change()
```

### Lógica de `enabled` vs `override_active`

| `enabled` | `override_active` | Comportamento |
|-----------|-------------------|---------------|
| `false` | qualquer | FSM pausada, hold setpoint a ≥20 Hz |
| `true` | `true` | FSM congelada, hold setpoint a 100 Hz |
| `true` | `false` | Operação normal da FSM |

> **Diferença chave:** `enabled=false` publica apenas na frequência mínima do watchdog (20 Hz). `override_active=true` publica a 100 Hz (cada ciclo do loop), pois a FSM ainda está "ativa" sob o ponto de vista do timer.

## 7. `odometry_callback()` — Atualização de Estado

Chamado a cada mensagem de odometria do MAVROS (tipicamente 50-100 Hz):

```cpp
void DroneControllerCompleto::odometry_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Posição NED (North-East-Down)
  current_x_ned_ = msg->pose.pose.position.x;
  current_y_ned_ = msg->pose.pose.position.y;
  current_z_ned_ = msg->pose.pose.position.z;
  // Z real = valor absoluto (NED: Z negativo é "cima")
  current_z_real_ = std::abs(current_z_ned_);

  // Velocidades lineares NED
  current_vx_ned_ = msg->twist.twist.linear.x;
  current_vy_ned_ = msg->twist.twist.linear.y;
  current_vz_ned_ = msg->twist.twist.linear.z;

  // Yaw (radianos) extraído do quaternion de orientação
  const auto & q = msg->pose.pose.orientation;
  current_yaw_rad_ = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
```

> **`current_z_real_`:** Usa `std::abs()` porque o MAVROS publica a posição em NED, onde Z negativo corresponde a altitude positiva. O valor absoluto dá a altitude acima do ponto de origem diretamente.

## 8. `yaw_override_callback()` — Override de Yaw Rate

```cpp
void DroneControllerCompleto::yaw_override_callback(
  const drone_control::msg::YawOverride::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->enable) {
    if (!yaw_override_enabled_) {
      // Captura posição atual como posição de hold (o drone gira no lugar)
      hold_x_ned_ = current_x_ned_;
      hold_y_ned_ = current_y_ned_;
      hold_z_ned_ = current_z_ned_;
      hold_valid_ = true;
    }
    yaw_override_enabled_ = true;
    yaw_rate_cmd_ = static_cast<double>(msg->yaw_rate);
    if (msg->timeout > 0.0f) {
      yaw_override_timeout_s_ = static_cast<double>(msg->timeout);
    }
  } else {
    yaw_override_enabled_ = false;
    yaw_rate_cmd_ = 0.0;
  }
  last_yaw_cmd_time_ = this->now();  // reinicia watchdog de timeout
}
```

**Exemplo de uso — girar o drone 0.5 rad/s por 2 segundos:**
```cpp
// Publicar no tópico /uav1/yaw_override/cmd
drone_control::msg::YawOverride msg;
msg.enable   = true;
msg.yaw_rate = 0.5f;   // rad/s (positivo = sentido anti-horário em NED)
msg.timeout  = 2.0f;   // após 2 s sem novo cmd, override é desativado automaticamente
publisher->publish(msg);
```
