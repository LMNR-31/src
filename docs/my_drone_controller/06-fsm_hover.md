# 06 — FSM Estado 2: Hover (`fsm_hover.cpp`)

O arquivo `src/fsm_hover.cpp` implementa o **Estado 2 (HOVER)** da FSM. Este é o estado de **espera ativa**: o drone mantém a altitude de decolagem enquanto aguarda o recebimento de waypoints de trajetória.

## 1. Visão Geral do Estado HOVER

O drone entra no estado HOVER imediatamente após atingir `takeoff_target_z_` (transição 1→2). Neste estado o nó:

1. **Publica setpoints de posição** continuamente na altitude atual para manter o hover.
2. **Monitora `ExtendedState`** do FCU para detectar pouso inadvertido.
3. **Aguarda `controlador_ativo_`** ser ativado (pelo recebimento de `/waypoints`).
4. Transita para **Estado 3 (TRAJECTORY)** quando `controlador_ativo_ = true`.
5. Transita para **Estado 4 (LANDING)** se `pouso_em_andamento_ = true`.

## 2. Código de `handle_state2_hover()`

```cpp
// Arquivo: src/fsm_hover.cpp
void DroneControllerCompleto::handle_state2_hover()
{
  // ── Publicação do setpoint de hover ────────────────────────────────────
  if (using_4d_goal_) {
    // Modo 4D: mantém posição + yaw absoluto configurado no goal
    publishPositionTargetWithYaw(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      last_waypoint_goal_.pose.position.z,
      goal_yaw_rad_);
  } else {
    // Modo 3D: mantém apenas posição (yaw livre)
    publishPositionTarget(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      last_waypoint_goal_.pose.position.z,
      0.0, MASK_POS_ONLY);
  }

  // Log periódico a cada 10 s (THROTTLE evita spam)
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "🛸 Em HOVER (%.1fm) | X=%.2f, Y=%.2f | Controlador: %s",
    last_waypoint_goal_.pose.position.z,
    last_waypoint_goal_.pose.position.x,
    last_waypoint_goal_.pose.position.y,
    controlador_ativo_ ? "ATIVO" : "INATIVO");

  // ── Detecção de pouso pelo autopiloto (via ExtendedState) ────────────────
  if (autopilot_indicates_landing()) {
    const uint8_t ls = last_extended_state_.landed_state;
    RCLCPP_WARN(this->get_logger(),
      "🛬 [HOVER] POUSO DETECTADO (autopiloto)! landed_state=%d, Z=%.2f m",
      static_cast<int>(ls), current_z_real_);
    trigger_landing(current_z_real_);
    return;
  }

  // ── Transição para Estado 3 (TRAJECTORY) ────────────────────────────────
  if (controlador_ativo_) {
    state_voo_ = 3;
    return;
  }

  // ── Fallback: pouso direto se sinalizdo ─────────────────────────────────
  if (pouso_em_andamento_) {
    state_voo_ = 4;
  }
}
```

## 3. Hover em Modo 3D vs. Modo 4D

### Modo 3D (`using_4d_goal_ = false`)

Quando o waypoint foi recebido via `/waypoint_goal` ou `/waypoints` (sem yaw), o hover usa `MASK_POS_ONLY`. O PX4 controla o heading autonomamente.

```cpp
publishPositionTarget(x, y, z, 0.0, MASK_POS_ONLY);
```

### Modo 4D (`using_4d_goal_ = true`)

Quando o waypoint foi recebido via `/waypoint_goal_4d` ou `/waypoints_4d` (com campo `yaw`), o hover usa `MASK_POS_YAW` e mantém o yaw absoluto especificado.

```cpp
publishPositionTargetWithYaw(x, y, z, goal_yaw_rad_);
```

**Exemplo:**
```
Waypoint 4D recebido: X=3.0, Y=2.0, Z=2.0, yaw=1.57 rad (≈ 90°, olhando para Norte)
→ Hover publica MASK_POS_YAW com yaw=1.57 rad
→ Drone mantém X=3.0, Y=2.0, Z=2.0 com heading fixo para Norte
```

## 4. Detecção de Pouso via `autopilot_indicates_landing()`

Esta função verifica se o FCU (via `ExtendedState`) indica que o drone está pousando ou já está no solo:

```cpp
bool DroneControllerCompleto::autopilot_indicates_landing() const
{
  // Só confia se já recebeu ao menos uma mensagem
  if (!extended_state_received_) { return false; }

  // Só confia se a mensagem é recente (< 1 s)
  const double age_s = (this->now() - last_extended_state_time_).seconds();
  if (age_s > 1.0) { return false; }

  // Só dispara quando armado (evita falso positivo no solo)
  if (!current_state_.armed) { return false; }

  // Verifica LANDING (descendo) ou ON_GROUND (aterrisado)
  const uint8_t ls = last_extended_state_.landed_state;
  return (ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING ||
          ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND);
}
```

### Valores de `landed_state`

| Valor | Constante MAVLink | Significado |
|-------|-------------------|-------------|
| 0 | `LANDED_STATE_UNDEFINED` | Estado desconhecido |
| 1 | `LANDED_STATE_ON_GROUND` | Drone confirmado no solo |
| 2 | `LANDED_STATE_IN_AIR` | Drone em voo normal |
| 3 | `LANDED_STATE_TAKEOFF` | Drone em processo de decolagem |
| 4 | `LANDED_STATE_LANDING` | Drone em processo de pouso |

A detecção no HOVER é acionada para valores **1** ou **4**.

## 5. Ativação da Trajetória via `controlador_ativo_`

A flag `controlador_ativo_` é definida como `true` em `activate_trajectory_in_hover()`, que é chamada por `waypoints_callback()` ou `waypoints_4d_callback()` quando:

- O drone já está no estado 2 (HOVER), **e**
- O array de waypoints tem 2 ou mais pontos.

```cpp
void DroneControllerCompleto::activate_trajectory_in_hover(size_t waypoint_count)
{
  // Confirma HOVER na fila de comandos
  if (hover_cmd_id_) {
    cmd_queue_.confirm(*hover_cmd_id_, true);
    hover_cmd_id_.reset();
  }
  // Enfileira TRAJECTORY
  trajectory_cmd_id_ = cmd_queue_.enqueue(
    CommandType::TRAJECTORY, {{"waypoints", std::to_string(waypoint_count)}});

  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
  state_voo_ = 3;  // transita imediatamente para TRAJECTORY
}
```

**Nota:** Se o array de waypoints chegar enquanto `state_voo_ != 2` (ex.: ainda no Estado 1 decolando), o sistema armazena os waypoints em `trajectory_waypoints_` e aguarda. Na próxima vez que o drone entrar no Estado 2, `handle_state2_hover()` vê `controlador_ativo_ = true` e transita para o Estado 3.

## 6. Diagrama de Transições do Estado HOVER

```
         Entra no Estado 2 (vindo do Estado 1)
                     │
                     ▼
         Publica setpoint de hover (100 Hz)
                     │
         ┌───────────┼──────────────────────────┐
         │           │                          │
         ▼           ▼                          ▼
  autopilot     controlador_ativo_         pouso_em_andamento_
  _indicates    = true                     = true
  _landing()    (recebeu /waypoints        (comando de pouso
  = true        com 2+ wps e está          detectado)
         │      no estado 2)                    │
         ▼           │                          │
  trigger_landing()  ▼                          ▼
  state_voo_ = 4     state_voo_ = 3       state_voo_ = 4
  (LANDING)          (TRAJECTORY)         (LANDING)
```

## 7. Exemplo de Cenário Completo: Takeoff + Hover + Trajetória

```
t=0s:  Recebe /waypoints com 3 waypoints (WP0, WP1, WP2)
       → trajectory_waypoints_ = [WP0, WP1, WP2]
       → state_voo_ = 2 (drone ainda está subindo)

t=5s:  Drone atinge hover_altitude → state_voo_ = 2 (HOVER)
       → handle_state2_hover() vê controlador_ativo_ = false
       → Publica hover setpoint, aguarda

t=6s:  /waypoints recebido de novo (ou já estava armazenado):
       activate_trajectory_in_hover(3) é chamado:
       → controlador_ativo_ = true
       → state_voo_ = 3 (TRAJECTORY)
       → log: "✈️ Iniciando execução de trajetória..."
```

## 8. Após Conclusão da Trajetória: Retorno ao Hover

Quando todos os waypoints são atingidos em `finalize_trajectory_complete()`:

```cpp
// Atualiza posição de hover para o último waypoint
last_waypoint_goal_.pose = trajectory_waypoints_.back();
// Volta para Estado 2
controlador_ativo_ = false;
state_voo_ = 2;
```

O drone então volta a publicar setpoints no último waypoint atingido (não na posição original de decolagem), aguardando novos `/waypoints`.
