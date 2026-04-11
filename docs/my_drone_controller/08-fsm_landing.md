# 08 — FSM Estado 4: Pouso (`fsm_landing.cpp`)

O arquivo `src/fsm_landing.cpp` implementa o **Estado 4 (LANDING)** da FSM. Este estado trata toda a sequência de pouso: aguarda a confirmação de aterrissagem, solicita DISARM ao FCU, aguarda confirmação e transita de volta ao **Estado 0 (WAIT)**, deixando o drone pronto para uma nova missão.

## 1. Funções implementadas

| Função | Descrição |
|--------|-----------|
| `handle_state4_disarm_reset()` | Verifica se o drone já desarmou enquanto ainda estava no Estado 4 |
| `complete_landing()` | Finaliza o pouso: confirma LAND, salva log e solicita DISARM |
| `handle_state4_landing()` | Handler principal do Estado 4, chamado a 100 Hz |

## 2. Como o Estado 4 é ativado

O Estado 4 é sempre iniciado via `trigger_landing()`, chamada por qualquer um dos estados quando pouso é detectado:

```cpp
void DroneControllerCompleto::trigger_landing(double z)
{
  pouso_em_andamento_ = true;
  state_voo_ = 4;
  // Enfileira comando LAND na fila de auditoria
  land_cmd_id_ = cmd_queue_.enqueue(
    CommandType::LAND, {{"z", std::to_string(z)}});
}
```

### Quem pode chamar `trigger_landing()`?

| Origem | Condição |
|--------|----------|
| `waypoints_callback()` | Waypoint único com Z < `land_z_threshold` |
| `waypoints_4d_callback()` | Waypoint 4D único com Z < `land_z_threshold` |
| `check_landing_in_flight()` | Estados 2/3, `autopilot_indicates_landing() = true` |
| `handle_state2_hover()` | `autopilot_indicates_landing() = true` no HOVER |
| `detect_and_handle_landing_in_trajectory()` | `autopilot_indicates_landing() = true` no TRAJECTORY |

## 3. `handle_state4_landing()` — Handler Principal

```cpp
void DroneControllerCompleto::handle_state4_landing()
{
  // ── 1. Aguardar confirmação de DISARM ───────────────────────────────────
  if (disarm_requested_) {
    if (!current_state_.armed) {
      // FCU confirmou DISARM: drone desarmado
      RCLCPP_INFO(this->get_logger(),
        "✅ DISARM confirmado pelo FCU — transicionando para aguardar novo comando.");
      disarm_requested_ = false;
      state_voo_ = 0;   // ← transita para Estado 0 (WAIT)
      return;
    } else {
      // Ainda aguardando DISARM (log a cada 3 s)
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "⏳ [DISARM] Aguardando confirmação de DISARM pelo FCU...");
    }
    return;
  }

  // ── 2. Log periódico de status ──────────────────────────────────────────
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "⬇️ POUSANDO — aguardando confirmação de aterrissagem...");

  // ── 3. Timeout de pouso ─────────────────────────────────────────────────
  if (pouso_em_andamento_) {
    if (!pouso_start_time_set_) {
      pouso_start_time_ = this->now();
      pouso_start_time_set_ = true;
    }

    if ((this->now() - pouso_start_time_).seconds() > config_.landing_timeout) {
      complete_landing();  // inicia DISARM após timeout
      return;
    }
  }
}
```

## 4. `complete_landing()` — Confirmação e DISARM

```cpp
void DroneControllerCompleto::complete_landing()
{
  // Confirma comando LAND na fila de auditoria
  if (land_cmd_id_) {
    cmd_queue_.confirm(*land_cmd_id_, true);
    land_cmd_id_.reset();
  }

  // Salva histórico de comandos em arquivo de log
  cmd_queue_.save_log("/tmp/drone_commands.log");

  // Solicita DISARM ao FCU
  request_disarm();
  disarm_requested_ = true;

  // Limpa flags de pouso (evita re-entrada neste bloco enquanto aguarda DISARM)
  pouso_em_andamento_ = false;
  pouso_start_time_set_ = false;
}
```

## 5. `handle_state4_disarm_reset()` — Tratamento de Chegada de Novo Comando

Esta função é chamada pelos callbacks de waypoint **antes** de processar o novo comando. Seu objetivo é lidar com a situação em que o drone pousou e desarmou **antes** de `handle_state4_landing()` detectar o DISARM:

```cpp
bool DroneControllerCompleto::handle_state4_disarm_reset()
{
  if (state_voo_ != 4) { return false; }

  if (!current_state_.armed) {
    // Drone já desarmou enquanto estava no Estado 4.
    // Transita para Estado 0 e retorna FALSE para que
    // o novo comando de takeoff seja processado imediatamente.
    RCLCPP_INFO(this->get_logger(),
      "✅ DRONE DESARMADO em estado 4 — transicionando para aguardar novo comando.");
    disarm_requested_ = false;
    state_voo_ = 0;
    return false;  // permite processamento do novo comando
  }

  if (disarm_requested_) {
    // DISARM ainda pendente: ignora o novo comando (pouso em andamento)
    RCLCPP_INFO(this->get_logger(),
      "[DISARM] Aguardando confirmação de DISARM; ignorando waypoint recebido.");
  }
  return true;  // bloqueia processamento do novo comando
}
```

### Tabela de respostas de `handle_state4_disarm_reset()`

| Estado | `armed` | `disarm_requested_` | Retorno | Efeito |
|--------|---------|---------------------|---------|--------|
| ≠ 4 | qualquer | qualquer | `false` | Não faz nada |
| 4 | `false` | qualquer | `false` | Transita para 0, aceita novo takeoff |
| 4 | `true` | `true` | `true` | Descarta o novo comando (aguarda DISARM) |
| 4 | `true` | `false` | `true` | Descarta o novo comando (ainda pousando) |

## 6. Diagrama de Fluxo do Estado 4

```
trigger_landing(z) é chamado
    │
    ▼
state_voo_ = 4
pouso_em_andamento_ = true
land_cmd_id_ = enqueued
    │
    ▼
handle_state4_landing() @ 100 Hz
    │
    ├── disarm_requested_ = true?
    │       ├── !armed → state_voo_ = 0  (FIM)
    │       └── armed → aguarda (log a cada 3s)
    │
    └── pouso_em_andamento_ = true?
            ├── !pouso_start_time_set_ → registra t_inicio
            │
            └── elapsed > landing_timeout (3 s)?
                    └── complete_landing()
                            ├── confirma LAND no cmd_queue
                            ├── save_log("/tmp/drone_commands.log")
                            ├── request_disarm()
                            └── disarm_requested_ = true
                                pouso_em_andamento_ = false
```

## 7. Fluxo de DISARM e transição para Estado 0

```
complete_landing() chama request_disarm()
    │
    ▼
request_disarm() envia async_send_request para
/uav1/mavros/cmd/arming com value=false
    │
    ▼
FCU processa e responde no callback:
    ├── success=true → cmd_queue_.confirm(DISARM, true)
    └── success=false → cmd_queue_.confirm(DISARM, false)
    │
    ▼
handle_state4_landing() @ próximos ciclos:
    disarm_requested_ = true
    └── !current_state_.armed  (FCU confirmou)
            → disarm_requested_ = false
            → state_voo_ = 0
            → "Aguardando novo comando de waypoint para decolar..."
```

## 8. Log de Comandos em `/tmp/drone_commands.log`

Ao final de cada pouso, `complete_landing()` salva o histórico completo de comandos da missão em `/tmp/drone_commands.log`. O arquivo contém uma linha por comando com formato:

```
[timestamp] ID=N | TIPO | STATUS | dados=...
Exemplo:
[2024-01-15 10:23:01] ID=1 | ARM               | CONFIRMADO | -
[2024-01-15 10:23:01] ID=2 | SET_MODE_OFFBOARD | CONFIRMADO | mode=OFFBOARD
[2024-01-15 10:23:05] ID=3 | TAKEOFF           | CONFIRMADO | x=5.00 y=3.00 z=2.00
[2024-01-15 10:23:12] ID=4 | HOVER             | CONFIRMADO | x=5.00 y=3.00 z=2.00
[2024-01-15 10:23:20] ID=5 | TRAJECTORY        | CONFIRMADO | waypoints=3
[2024-01-15 10:23:45] ID=6 | LAND              | CONFIRMADO | z=0.00
[2024-01-15 10:23:48] ID=7 | DISARM            | CONFIRMADO | -
```

## 9. Exemplo de Sequência de Logs do Estado 4

```
[WARN]  🛬🛬🛬 POUSO DETECTADO! Z_final = 0.00 m
[WARN]  📋 [ID=6] Comando LAND enfileirado
[WARN]  🛬 POUSANDO — delegando descida ao drone_soft_land

[INFO]  ⬇️ POUSANDO — aguardando confirmação de aterrissagem...
[INFO]  ⏱️ Iniciando contagem de pouso (3 s para confirmar)...

(3 segundos depois...)

[WARN]  ✅ [ID=6] LAND confirmado — iniciando DISARM
[INFO]  💾 Histórico de comandos salvo em /tmp/drone_commands.log
[WARN]  ✅ POUSO CONCLUÍDO! Solicitando DISARM e aguardando confirmação do FCU...
[INFO]  🔴 [ID=7] Solicitando DISARM...
[INFO]  ✅ [ID=7] DISARM confirmado pelo FCU

[INFO]  ✅ DISARM confirmado pelo FCU — transicionando para aguardar novo comando.
[WARN]  ⏳ Aguardando novo comando de waypoint para decolar novamente...
```
