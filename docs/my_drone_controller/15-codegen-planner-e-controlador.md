# 15 — Planner e Controlador Codegen (`TrajectoryPlanner_codegen` + `Drone_codegen`)

> **Arquivos:**
> - `src/TrajectoryPlanner_codegen.cpp` — gerador de setpoints de trajetória polinomial
> - `src/Drone_codegen.cpp` — controlador PID de posição
> - `include/TrajectoryPlanner_codegen.h` e `include/Drone_codegen.h` — cabeçalhos

Estes dois módulos foram gerados a partir de código MATLAB e adaptados para ROS 2. Eles trabalham **em conjunto** dentro do `handle_state3_trajectory()` para gerar setpoints suaves de posição e velocidade.

---

## 1. Visão Geral: Como os dois módulos se encaixam

```
FSM Estado 3 — a cada 10 ms:
  │
  ├─ TrajectoryPlanner_codegen::getNextSetpoint(t, Xd, Vd, Ad)
  │     └─ Calcula: Xd = posição desejada no tempo t
  │                 Vd = velocidade desejada (feedforward)
  │                 Ad = aceleração desejada (feedforward)
  │
  ├─ Drone_codegen::PositionCtrl(Xd, Vd, Ad)
  │     ├─ Lê posição atual (r[]) e velocidade atual (dr[]) da odometria
  │     ├─ Calcula erro de posição e velocidade
  │     └─ Gera: zdot_des = velocidade vertical desejada (PID Z)
  │              theta_des, phi_des = atitude desejada (roll/pitch)
  │
  └─ publishPositionTargetWithVelocityAndYaw(
         Xd[0], Xd[1], Xd[2],     ← posição do planner
         Vd[0], Vd[1],             ← velocidade XY do planner (feedforward)
         drone_ctrl_.zdot_des,      ← velocidade Z do controlador (PID)
         yaw_follow)               ← yaw calculado pela FSM
```

**Design:** XY usa feedforward do planner (trajetória suave), Z usa PID (corrige erros de altitude ativamente).

---

## 2. `TrajectoryPlanner_codegen` — Planner Polinomial

### 2.1 Estrutura de Dados (do cabeçalho)

```cpp
class TrajectoryPlanner_codegen {
public:
  // ── Entradas (configuradas antes de init()) ──────────────────────────────
  std::vector<double> waypoints;    // [X0..Xn, Y0..Yn, Z0..Zn] — flat array
  std::vector<double> segmentTimes; // duração de cada segmento (ex.: {5.0, 5.0, 5.0})
  int numSegments;                  // n = número de segmentos = nWaypoints - 1

  // ── Saídas internas ──────────────────────────────────────────────────────
  coder::array<double, 3U> coefficients;  // [n × 3 × 6] — coeficientes polinomiais
  double X_final[3];    // posição do último waypoint (trava quando t > total_time)

  // ── Estado ───────────────────────────────────────────────────────────────
  int currentSegment;   // segmento atual (0-indexed)
  double tAccum;        // tempo acumulado (atualmente não usado no getNextSetpoint)
  bool inHover;         // flag de hover (atualmente não utilizada)

  TrajectoryPlanner_codegen* init();
  void getNextSetpoint(double t, double Xd[3], double Vd[3], double Ad[3]);
};
```

### 2.2 Formato do array `waypoints`

O array é **flat** (unidimensional), organizado como:
```
waypoints = [X0, X1, X2, ..., Xn,   ← posições X dos n+1 waypoints
             Y0, Y1, Y2, ..., Yn,   ← posições Y dos n+1 waypoints
             Z0, Z1, Z2, ..., Zn]   ← posições Z dos n+1 waypoints
```

Exemplo com 3 waypoints (n=2 segmentos):
```cpp
// WP0=(0,0,2), WP1=(3,0,2), WP2=(3,3,2)
planner_.waypoints = {0.0, 3.0, 3.0,   // X0, X1, X2
                      0.0, 0.0, 3.0,   // Y0, Y1, Y2
                      2.0, 2.0, 2.0};  // Z0, Z1, Z2
planner_.segmentTimes = {5.0, 5.0};    // 5 s para WP0→WP1, 5 s para WP1→WP2
planner_.numSegments  = 2;
```

### 2.3 `init()` — Pré-computar coeficientes polinomiais

```cpp
TrajectoryPlanner_codegen *TrajectoryPlanner_codegen::init()
{
  this->currentSegment = 0;   // inicia no primeiro segmento
  this->tAccum         = 0;   // reinicia tempo acumulado
  this->inHover        = false;

  int n = this->numSegments;  // número de segmentos (lido de quem configurou)

  // Aloca matriz 3D de coeficientes: [n segmentos × 3 eixos × 6 coeficientes]
  // O polinômio de ordem 5 tem 6 coeficientes: a0..a5
  this->coefficients.set_size(n, 3, 6);

  // ── Cálculo dos coeficientes para cada segmento e eixo ─────────────────
  for (int seg = 0; seg < n; seg++) {
    double T = this->segmentTimes[seg];  // duração do segmento seg (em segundos)

    for (int dim = 0; dim < 3; dim++) {  // dim: 0=X, 1=Y, 2=Z
      int total_pontos = n + 1;          // n+1 waypoints no total

      // Lê ponto inicial e final do segmento para este eixo:
      //   waypoints[seg + dim * total_pontos]     = ponto A
      //   waypoints[(seg+1) + dim * total_pontos] = ponto B
      double p_start = this->waypoints[seg + dim * total_pontos];
      double p_end   = this->waypoints[(seg + 1) + dim * total_pontos];

      // Polinômio linear (ordem 1): X(t) = a0 + a1*t
      //   a0 = posição inicial
      //   a1 = velocidade constante = (p_end - p_start) / T
      // Os coeficientes a2..a5 são zero (trajetória linear, não cúbica/quíntica)
      double a0 = p_start;
      double a1 = (p_end - p_start) / T;

      // Índice na matriz flat 3D [n × 3 × 6]:
      //   coef k, eixo dim, segmento seg → índice = seg + dim*n + k*(n*3)
      this->coefficients[seg + dim * n + 0 * (n * 3)] = a0;  // coef constante
      this->coefficients[seg + dim * n + 1 * (n * 3)] = a1;  // coef linear
      this->coefficients[seg + dim * n + 2 * (n * 3)] = 0.0; // coef quadrático
      this->coefficients[seg + dim * n + 3 * (n * 3)] = 0.0; // coef cúbico
      this->coefficients[seg + dim * n + 4 * (n * 3)] = 0.0; // coef quártico
      this->coefficients[seg + dim * n + 5 * (n * 3)] = 0.0; // coef quíntico
    }
  }

  // Salva posição final de segurança (trava no último waypoint após o tempo total):
  this->X_final[0] = this->waypoints[n];                  // último X
  this->X_final[1] = this->waypoints[n + (n + 1)];        // último Y
  this->X_final[2] = this->waypoints[n + 2 * (n + 1)];   // último Z

  return this;
}
```

### 2.4 `getNextSetpoint()` — consultar posição/velocidade/aceleração em t

```cpp
void TrajectoryPlanner_codegen::getNextSetpoint(
  double t,       // tempo decorrido desde o início da trajetória (segundos)
  double Xd[3],   // saída: posição desejada [X, Y, Z]
  double Vd[3],   // saída: velocidade desejada [Vx, Vy, Vz]
  double Ad[3])   // saída: aceleração desejada [Ax, Ay, Az]
{
  int n = this->numSegments;

  // ── Passo 1: determinar em qual segmento estamos ─────────────────────────
  int seg = 0;
  double t_start = 0.0;    // início do segmento atual no tempo global
  for (int i = 0; i < n; i++) {
    // +0.001 de margem para evitar erro de arredondamento de ponto flutuante
    if (t <= t_start + this->segmentTimes[i] + 0.001) {
      seg = i;       // estamos no segmento i
      break;
    }
    t_start += this->segmentTimes[i];
    if (i == n - 1) {
      seg = n - 1;   // se ultrapassou tudo, trava no último segmento
    }
  }

  // ── Passo 2: verificar se o tempo total foi ultrapassado ─────────────────
  double total_time = 0;
  for (int i = 0; i < n; i++) { total_time += this->segmentTimes[i]; }

  if (t >= total_time) {
    // Retorna a posição final fixa (sem velocidade/aceleração)
    Xd[0] = this->X_final[0]; Xd[1] = this->X_final[1]; Xd[2] = this->X_final[2];
    Vd[0] = 0.0; Vd[1] = 0.0; Vd[2] = 0.0;
    Ad[0] = 0.0; Ad[1] = 0.0; Ad[2] = 0.0;
    return;
  }

  // ── Passo 3: tempo local dentro do segmento ──────────────────────────────
  double t_seg = t - t_start;  // t_seg ∈ [0.0, segmentTimes[seg]]

  // ── Passo 4: avaliar polinômio para X, Y, Z ──────────────────────────────
  for (int dim = 0; dim < 3; dim++) {
    // Lê os 6 coeficientes do polinômio para este segmento e eixo:
    double a0 = this->coefficients[seg + dim * n + 0 * (n * 3)];
    double a1 = this->coefficients[seg + dim * n + 1 * (n * 3)];
    double a2 = this->coefficients[seg + dim * n + 2 * (n * 3)];
    double a3 = this->coefficients[seg + dim * n + 3 * (n * 3)];
    double a4 = this->coefficients[seg + dim * n + 4 * (n * 3)];
    double a5 = this->coefficients[seg + dim * n + 5 * (n * 3)];

    // Posição: X(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    Xd[dim] = a0 + a1*t_seg
              + a2*pow(t_seg,2) + a3*pow(t_seg,3)
              + a4*pow(t_seg,4) + a5*pow(t_seg,5);

    // Velocidade: V(t) = dX/dt = a1 + 2*a2*t + 3*a3*t² + 4*a4*t³ + 5*a5*t⁴
    Vd[dim] = a1 + 2*a2*t_seg
              + 3*a3*pow(t_seg,2) + 4*a4*pow(t_seg,3)
              + 5*a5*pow(t_seg,4);

    // Aceleração: A(t) = dV/dt = 2*a2 + 6*a3*t + 12*a4*t² + 20*a5*t³
    Ad[dim] = 2*a2 + 6*a3*t_seg
              + 12*a4*pow(t_seg,2) + 20*a5*pow(t_seg,3);
  }
}
```

### Exemplo numérico (trajetória linear)

```
Segmento 0: WP0=(0,0,2) → WP1=(3,0,2), T=5s
  a0_x=0.0, a1_x=(3-0)/5=0.6  (todos a2..a5 = 0)

t=2.5s (meio do segmento):
  Xd[0] = 0.0 + 0.6*2.5 = 1.5 m    ← metade do caminho em X
  Vd[0] = 0.6 m/s                   ← velocidade constante
  Ad[0] = 0.0 m/s²                  ← sem aceleração (trajetória linear)
```

---

## 3. `Drone_codegen` — Controlador PID de Posição

### 3.1 Estrutura de Dados

```cpp
class Drone_codegen {
public:
  // ── Parâmetros físicos ───────────────────────────────────────────────────
  double g   = 9.81;   // aceleração gravitacional (m/s²)
  double dt  = 0.01;   // passo de integração = 10 ms (100 Hz)
  double m   = 1.25;   // massa do drone (kg)

  // ── Entradas (atualizadas a cada ciclo pelo handle_state3_trajectory) ────
  double r[3];    // posição atual [X, Y, Z] em NED (da odometria)
  double dr[3];   // velocidade atual [Vx, Vy, Vz] em NED (da odometria)
  double euler[3]; // atitude atual [roll, pitch, yaw] (não usado no controle de posição)
  double w[3];    // velocidades angulares (não usado atualmente)

  // ── Saídas ───────────────────────────────────────────────────────────────
  double zdot_des;    // velocidade vertical desejada (m/s) — usada no setpoint Z
  double theta_des;   // ângulo de pitch desejado (rad) — para referência
  double phi_des;     // ângulo de roll desejado (rad) — para referência
  double r_des[3];    // posição desejada (cópia de Xd)
  double dr_des[3];   // velocidade desejada (cópia de Vd)
  double r_err[3];    // erro de posição = r_des - r

  // ── Ganhos PID ───────────────────────────────────────────────────────────
  double kP_pos[3], kI_pos[3], kD_pos[3];  // posição XYZ
  double kP_zdot, kI_zdot;                  // velocidade Z
  double kP_phi, kD_phi;                    // roll
  double kP_theta, kD_theta;                // pitch

  // ── Estado interno do integrador ─────────────────────────────────────────
  double r_err_sum[3];    // soma acumulada do erro de posição (para o I do PID)
  double zdot_err_sum;    // soma acumulada do erro de zdot
  double zdot_err_prev;   // erro de zdot no ciclo anterior (para derivativa)
  double phi_err_sum, theta_err_sum, psi_err_sum;  // integradores de atitude

  Drone_codegen* init();
  Drone_codegen* PositionCtrl(const double Xd[3], const double Vd[3], const double Ad[3]);
};
```

### 3.2 `init()` — Inicialização

```cpp
Drone_codegen *Drone_codegen::init()
{
  Drone_codegen *obj = this;

  // ── Parâmetros físicos ───────────────────────────────────────────────────
  obj->g  = 9.81;   // gravidade (m/s²)
  obj->dt = 0.01;   // passo = 1 ciclo do control_loop a 100 Hz
  obj->m  = 1.25;   // massa do drone (kg)

  // ── Zeragem do estado dos integradores ──────────────────────────────────
  // É crítico zerar os integradores a cada nova trajetória para evitar
  // que erros acumulados de trajetórias anteriores contaminem a nova.
  for (int i = 0; i < 3; i++) {
    obj->r[i]         = 0.0;  // posição atual (será preenchida pela odometria)
    obj->dr[i]        = 0.0;  // velocidade atual
    obj->euler[i]     = 0.0;  // atitude atual
    obj->w[i]         = 0.0;  // velocidades angulares
    obj->r_err_sum[i] = 0.0;  // integrador de posição (para o termo I do PID)
  }
  obj->phi_err_sum   = 0.0;   // integrador de roll
  obj->theta_err_sum = 0.0;   // integrador de pitch
  obj->psi_err_sum   = 0.0;   // integrador de yaw
  obj->zdot_err_sum  = 0.0;   // integrador de velocidade Z
  obj->zdot_err_prev = 0.0;   // erro de zdot anterior (para derivativa)

  // ── Ganhos de atitude (roll/pitch) ──────────────────────────────────────
  obj->kP_phi   = 4.5;  obj->kD_phi   = 0.4;
  obj->kP_theta = 4.5;  obj->kD_theta = 0.4;

  // ── Ganhos de velocidade vertical (Z-dot) ───────────────────────────────
  obj->kP_zdot = 6.0;   // ganho proporcional
  obj->kI_zdot = 0.1;   // ganho integral (evita erro estacionário de altitude)

  // ── Ganhos de posição XYZ (PID completo) ────────────────────────────────
  //   Eixos X e Y são idênticos (sistema simétrico no plano horizontal)
  obj->kP_pos[0] = 4.2;  obj->kI_pos[0] = 0.93;  obj->kD_pos[0] = 3.0;  // X
  obj->kP_pos[1] = 4.2;  obj->kI_pos[1] = 0.93;  obj->kD_pos[1] = 3.0;  // Y
  obj->kP_pos[2] = 5.0;  obj->kI_pos[2] = 1.0;   obj->kD_pos[2] = 3.0;  // Z

  return obj;
}
```

### 3.3 `PositionCtrl()` — Controlador PID

```cpp
Drone_codegen *Drone_codegen::PositionCtrl(
  const double Xd[3],  // posição desejada [Xd, Yd, Zd] (do planner)
  const double Vd[3],  // velocidade desejada [Vxd, Vyd, Vzd] (do planner)
  const double Ad[3])  // aceleração desejada [Axd, Ayd, Azd] (do planner)
{
  Drone_codegen *obj = this;
  double p_cmd = obj->dt;   // dt = 0.01 s — usado na integração (soma * dt)
  double dr_err[3];         // erro de aceleração desejada (inclui PID + feedforward)

  // ── Laço PID de posição para X, Y, Z ────────────────────────────────────
  for (int i = 0; i < 3; i++) {
    obj->r_des[i]  = Xd[i];             // guarda posição desejada
    obj->dr_des[i] = Vd[i];             // guarda velocidade desejada

    // Erro de posição: e_pos = posição_desejada - posição_atual
    obj->r_err[i] = obj->r_des[i] - obj->r[i];

    // Erro de velocidade: e_vel = velocidade_desejada - velocidade_atual
    double vel_error = obj->dr_des[i] - obj->dr[i];

    // Integração do erro de posição: soma += e_pos * dt
    // Esse integral combate o erro estacionário (ex.: drone que não alcança exatamente a meta)
    obj->r_err_sum[i] += obj->r_err[i] * p_cmd;

    // PID completo → aceleração desejada (dr_err):
    //   P: kP * e_pos                (resposta proporcional ao erro)
    //   I: kI * integral(e_pos)      (elimina erro estacionário)
    //   D: kD * e_vel                (amortece oscilações)
    //   FF: Ad[i]                    (feedforward de aceleração do planner)
    dr_err[i] = (obj->kP_pos[i] * obj->r_err[i]
               + obj->kI_pos[i] * obj->r_err_sum[i]
               + obj->kD_pos[i] * vel_error)
               + Ad[i];  // feedforward — "sabe" a aceleração da trajetória
  }

  // ── Mapeamento para atitude: aceleração → ângulos ────────────────────────
  // Para um multirotor, a aceleração lateral é gerada inclinando o drone:
  //   ax → theta_des (pitch): inclinar para frente/trás produz aceleração em X
  //   ay → phi_des (roll):    inclinar para os lados produz aceleração em Y
  // A relação é: a_x ≈ g * theta   →   theta = -a_x / g
  //              a_y ≈ g * phi     →   phi   =  a_y / g
  obj->theta_des = -dr_err[0] / obj->g;   // pitch desejado (rad)
  obj->phi_des   =  dr_err[1] / obj->g;   // roll desejado (rad)

  // ── Velocidade vertical desejada (Z-dot) ─────────────────────────────────
  // Usa apenas os termos P e feedforward (não I nem D, para simplicidade):
  //   zdot_des = Vzd + kP_pos[2] * e_pos_z
  obj->zdot_des = Vd[2] + obj->kP_pos[2] * obj->r_err[2];

  // ── Saturação de atitude (segurança): máximo ±45° = ±0.785 rad ──────────
  // Evita que o PID solicite inclinações extremas que destabilizariam o drone.
  obj->phi_des   = std::max(std::min(obj->phi_des,   0.785), -0.785);
  obj->theta_des = std::max(std::min(obj->theta_des, 0.785), -0.785);

  return obj;
}
```

---

## 4. Como os dois módulos são usados em `handle_state3_trajectory()`

```cpp
// Em drone_controller_completo.cpp — dentro do if (planner_initialized_):

// 1. Tempo decorrido desde o início da trajetória
double elapsed = (this->now() - trajectory_start_time_).seconds();

// 2. Planner gera setpoint no tempo t=elapsed
double Xd[3], Vd[3], Ad[3];
planner_.getNextSetpoint(elapsed, Xd, Vd, Ad);

// 3. Alimenta posição/velocidade atual da odometria no controlador
drone_ctrl_.r[0]  = current_x_ned_;   // posição atual X
drone_ctrl_.r[1]  = current_y_ned_;   // posição atual Y
drone_ctrl_.r[2]  = current_z_ned_;   // posição atual Z
drone_ctrl_.dr[0] = current_vx_ned_;  // velocidade atual Vx
drone_ctrl_.dr[1] = current_vy_ned_;  // velocidade atual Vy
drone_ctrl_.dr[2] = current_vz_ned_;  // velocidade atual Vz

// 4. Controlador PID calcula comando de velocidade Z e atitudes
drone_ctrl_.PositionCtrl(Xd, Vd, Ad);
// Após esta chamada: drone_ctrl_.zdot_des está disponível

// 5. Publica setpoint composto:
publishPositionTargetWithVelocityAndYaw(
  Xd[0], Xd[1], Xd[2],        // posição do planner (MASK_POS)
  Vd[0], Vd[1],                // velocidade XY do planner (feedforward MASK_VEL)
  drone_ctrl_.zdot_des,        // velocidade Z do PID (corrige erros de altitude)
  yaw_follow);                 // yaw calculado para apontar ao próximo WP
```

---

## 5. Por que XY usa feedforward e Z usa PID?

| Eixo | Fonte do comando | Razão |
|---|---|---|
| X, Y (posição) | `Xd[0], Xd[1]` do planner | Trajetória suave pré-calculada |
| X, Y (velocidade) | `Vd[0], Vd[1]` do planner | Feedforward: PX4 antecipa o movimento |
| Z (posição) | `Xd[2]` do planner | Referência de altitude |
| Z (velocidade) | `drone_ctrl_.zdot_des` do PID | Corrige erro de altitude ativamente |

O Z é controlado separadamente porque:
1. Erros de altitude tendem a ser maiores (efeito do empuxo residual, vento vertical).
2. O feedforward Z do planner (`Vd[2]`) é pequeno (trajetórias geralmente horizontais).
3. O PID de Z garante que a altitude seja mantida mesmo com perturbações externas.

---

## 6. Diagrama de Fluxo de Sinal

```
Odometria ─────────────────► drone_ctrl_.r[]
                              drone_ctrl_.dr[]
                                     │
                                     ▼
Planner ──► Xd, Vd, Ad ────► PositionCtrl(Xd, Vd, Ad)
                │                    │
                │          ┌─────────┴──────────┐
                │          │                    │
                ▼          ▼                    ▼
           publishPositionTargetWithVelocityAndYaw()
             pos_x = Xd[0]    vel_x = Vd[0]    yaw = yaw_follow
             pos_y = Xd[1]    vel_y = Vd[1]
             pos_z = Xd[2]    vel_z = zdot_des (PID)
                │
                ▼
           /uav1/mavros/setpoint_raw/local
                │
                ▼
           PX4 FCU → motores
```
