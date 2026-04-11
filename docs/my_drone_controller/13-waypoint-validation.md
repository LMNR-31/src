# 13 — Validação de Waypoints (`waypoint_validation.cpp`)

> **Arquivos:** `src/waypoint_validation.cpp` e `include/my_drone_controller/waypoint_validation.hpp`

Este documento explica **linha a linha** o módulo de validação de waypoints — a camada de segurança que impede o controlador de aceitar coordenadas inválidas (NaN, Inf) ou fora dos limites físicos configurados.

---

## 1. Cabeçalho — `waypoint_validation.hpp`

```cpp
#pragma once
// Evita inclusão dupla deste header (alternativa moderna a #ifndef guard).

#include "geometry_msgs/msg/pose.hpp"
// Tipo Pose (position.x/y/z + orientation) — usado em validate_pose().

#include "geometry_msgs/msg/pose_stamped.hpp"
// Tipo PoseStamped (header + pose) — usado em validate_waypoint().

#include "drone_config.h"
// Struct DroneConfig com os limites de validação:
//   config.land_z_threshold   — Z abaixo deste valor = intenção de pouso
//   config.min_altitude       — altitude mínima de voo (ex.: 0.2 m)
//   config.max_altitude       — altitude máxima de voo (ex.: 500.0 m)
//   config.max_waypoint_distance — distância XY máxima da origem (ex.: 1000.0 m)

namespace drone_control {

/// @brief Valida um PoseStamped como waypoint seguro.
/// @return true se o waypoint é válido para uso; false caso contrário.
bool validate_waypoint(const geometry_msgs::msg::PoseStamped & msg,
                       const DroneConfig & config);

/// @brief Wrapper para Pose simples (sem header).
/// @return true se a pose é válida; false caso contrário.
bool validate_pose(const geometry_msgs::msg::Pose & pose,
                   const DroneConfig & config);

}  // namespace drone_control
```

---

## 2. Implementação — `waypoint_validation.cpp`

### Código completo anotado

```cpp
#include "my_drone_controller/waypoint_validation.hpp"
// Inclui as declarações de validate_waypoint e validate_pose.

#include "rclcpp/rclcpp.hpp"
// Necessário para RCLCPP_WARN (log de rejeição).

#include <cmath>
// Fornece std::isnan() e std::isinf() para verificar valores especiais IEEE 754.

namespace drone_control {

// ── validate_waypoint ────────────────────────────────────────────────────────
bool validate_waypoint(const geometry_msgs::msg::PoseStamped & msg,
                       const DroneConfig & config)
{
  // Extrai referência para a posição (evita repetir msg.pose.position...)
  const auto & pos = msg.pose.position;

  // ── VERIFICAÇÃO 1: NaN (Not-a-Number) ──────────────────────────────────
  // NaN ocorre em operações como 0/0, sqrt(-1) ou campos não inicializados.
  // Uma coordenada NaN enviaria o PX4 para uma posição indefinida — perigoso.
  if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z)) return false;

  // ── VERIFICAÇÃO 2: Infinito ─────────────────────────────────────────────
  // Inf ocorre em divisões por zero (ex.: 1.0/0.0) ou overflow de float.
  // Também resultaria em um setpoint inválido para o FCU.
  if (std::isinf(pos.x) || std::isinf(pos.y) || std::isinf(pos.z)) return false;

  // ── VERIFICAÇÃO 3: Altitude (Z) — apenas para waypoints acima do threshold ─
  // Se z < land_z_threshold, o código interpreta como INTENÇÃO DE POUSO,
  // então não verifica min/max_altitude — o valor de Z serve como altitude-alvo
  // de descida, não como altitude de voo. O pouso pode ter Z negativo (solo).
  //
  // Se z >= land_z_threshold, o waypoint é de VOO → aplica limites físicos:
  if (pos.z >= config.land_z_threshold) {

    // Z abaixo da altitude mínima de voo (ex.: 0.2 m): rejeita.
    // Isso evita colisões com o solo ao tentar voar rente ao chão.
    if (pos.z < config.min_altitude) {
      RCLCPP_WARN(rclcpp::get_logger("validate_waypoint"),
        "❌ Waypoint Z=%.2fm REJEITADO: abaixo da altitude mínima (%.2fm)",
        pos.z, config.min_altitude);
      return false;
    }

    // Z acima da altitude máxima configurada (ex.: 500.0 m): rejeita.
    // Protege contra comandos acidentais de altitude extrema.
    if (pos.z > config.max_altitude) {
      RCLCPP_WARN(rclcpp::get_logger("validate_waypoint"),
        "❌ Waypoint Z=%.2fm REJEITADO: acima da altitude máxima (%.2fm)",
        pos.z, config.max_altitude);
      return false;
    }
  }

  // ── VERIFICAÇÃO 4: Distância XY ─────────────────────────────────────────
  // Rejeita waypoints fora da área de operação configurada.
  // max_waypoint_distance é a distância máxima em X ou Y individualmente
  // (não distância euclidiana, mas ±limite em cada eixo).
  //
  // Exemplo: max_waypoint_distance=1000.0 → aceita qualquer X,Y ∈ [-1000, 1000] m.
  if (std::abs(pos.x) > config.max_waypoint_distance) return false;
  if (std::abs(pos.y) > config.max_waypoint_distance) return false;

  // ── Waypoint aprovado em todos os checks ─────────────────────────────────
  return true;
}

// ── validate_pose ────────────────────────────────────────────────────────────
bool validate_pose(const geometry_msgs::msg::Pose & pose,
                   const DroneConfig & config)
{
  // Cria um PoseStamped temporário apenas para reutilizar validate_waypoint().
  // O header (stamp, frame_id) não é usado na validação — só a posição importa.
  geometry_msgs::msg::PoseStamped ps;
  ps.pose = pose;        // copia a pose para o campo .pose do PoseStamped
  return validate_waypoint(ps, config);   // delega para validate_waypoint
}

}  // namespace drone_control
```

---

## 3. Fluxo de Decisão

```
Recebe pos (x, y, z)
  │
  ├─ isnan(x) ou isnan(y) ou isnan(z)  → REJEITA (false)
  ├─ isinf(x) ou isinf(y) ou isinf(z) → REJEITA (false)
  │
  ├─ z >= land_z_threshold ?
  │     ├─ z < min_altitude  → REJEITA (false) + WARN
  │     └─ z > max_altitude  → REJEITA (false) + WARN
  │
  ├─ abs(x) > max_waypoint_distance → REJEITA (false)
  ├─ abs(y) > max_waypoint_distance → REJEITA (false)
  │
  └─ todos os checks passaram  → ACEITA (true)
```

---

## 4. Valores Padrão dos Limites (de `drone_config.h`)

| Parâmetro | Valor padrão | Significado |
|---|---|---|
| `land_z_threshold` | `0.3` m | Z abaixo disto = intenção de pouso |
| `min_altitude` | `0.2` m | Altitude mínima de voo seguro |
| `max_altitude` | `500.0` m | Teto máximo de operação |
| `max_waypoint_distance` | `1000.0` m | Distância XY máxima da origem |

Esses valores podem ser sobrescritos no arquivo `config/drone_config.yaml` ou via linha de comando com `--ros-args`.

---

## 5. Onde `validate_waypoint` e `validate_pose` são chamados

| Callback/Função | Função de validação | O que é validado |
|---|---|---|
| `waypoints_callback()` | `validate_pose()` | Cada elemento de `msg->poses` |
| `waypoint_goal_callback()` | `validate_waypoint()` | O `PoseStamped` recebido |
| `waypoints_4d_callback()` | `validate_waypoint()` | Cada `wp.pose` do array 4D |
| `waypoint_goal_4d_callback()` | `validate_waypoint()` | O `Waypoint4D.pose` recebido |

---

## 6. Exemplo: detecção de waypoint de pouso vs. voo

```
Configuração: land_z_threshold=0.3, min_altitude=0.2, max_altitude=500.0

Caso 1: pos=(1.0, 2.0, 2.5)
  z=2.5 >= 0.3 → verifica altitude: 0.2 ≤ 2.5 ≤ 500 → OK
  abs(1.0) ≤ 1000, abs(2.0) ≤ 1000 → OK
  Resultado: true (waypoint de voo válido)

Caso 2: pos=(0.0, 0.0, -0.1)
  z=-0.1 < 0.3 → skip altitude checks (é intenção de pouso)
  abs(0.0) ≤ 1000, abs(0.0) ≤ 1000 → OK
  Resultado: true (waypoint de pouso válido)

Caso 3: pos=(0.5, 0.5, 0.1)
  z=0.1 >= 0.3? Não → skip altitude checks
  Resultado: true (interpretado como altitude de pouso)

Caso 4: pos=(nan, 1.0, 2.0)
  isnan(nan) = true → Resultado: false (rejeitado)

Caso 5: pos=(1500.0, 0.0, 2.0)
  abs(1500.0) > 1000.0 → Resultado: false (fora da área de operação)
```

---

## 7. Por que usar `validate_pose` e `validate_waypoint` separados?

- `PoseArray` (waypoints de trajetória) contém `Pose` simples (sem header), por isso `validate_pose` é o wrapper conveniente que cria um `PoseStamped` temporário.
- `PoseStamped` (waypoint_goal) já tem o header, então `validate_waypoint` é chamado diretamente.
- Manter a lógica real em um único lugar (`validate_waypoint`) garante que qualquer mudança de regra se propague para todos os usos.
