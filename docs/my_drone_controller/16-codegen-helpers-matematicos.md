# 16 — Helpers Matemáticos Codegen (`cos`, `sqrt`, `minOrMax`, `mldivide`, `rt_nonfinite`)

> **Arquivos cobertos neste documento:**
>
> | Arquivo fonte | Cabeçalho | Tamanho funcional |
> |---------------|-----------|-------------------|
> | `src/cos.cpp` | `include/cos.h` | stub vazio |
> | `src/sqrt.cpp` | `include/sqrt.h` | stub vazio |
> | `src/minOrMax.cpp` | `include/minOrMax.h` | 4 funções de saturação |
> | `src/mldivide.cpp` | `include/mldivide.h` | 3 sobrecargas LU |
> | `src/rt_nonfinite.cpp` | `include/rt_nonfinite.h` | 6 constantes globais |
>
> Todos foram **gerados pelo MATLAB Coder 23.2** em 10-Mar-2026 e adaptados ao
> build ROS 2 do pacote `my_drone_controller`.

---

## 1. Papel / Responsabilidade no Sistema

Estes cinco arquivos formam a **biblioteca de suporte matemático** (runtime de
codegen) sobre a qual `Drone_codegen.cpp` e `TrajectoryPlanner_codegen.cpp` são
construídos. Eles **não contêm lógica de controle ou de FSM** — sua função é
prover primitivas matemáticas numéricas corretas (saturação, solução de sistemas,
constantes especiais) para que os módulos de alto nível operem sem depender do
runtime proprietário MATLAB.

```
Drone_codegen.cpp  ──┐
                      ├─► minOrMax.h   (saturação de velocidades/acelerações)
TrajectoryPlanner ───┤    mldivide.h   (ajuste polinomial via sistema linear)
  _codegen.cpp    ───┤    rt_nonfinite.h (sentinelas NaN / ±Inf)
                      └─► cos.h, sqrt.h (stubs de compatibilidade)
```

> **Impacto na FSM:** indireto. Erros nestes helpers afetam os setpoints gerados
> durante o **Estado 3 (trajetória)**, mas não alteram variáveis de FSM diretamente.

---

## 2. `rt_nonfinite.cpp` — Constantes Globais NaN / ±Inf

### 2.1 Papel

Inicializa seis variáveis globais que representam valores especiais IEEE-754
(`NaN`, `+Inf`, `-Inf`) em precisão dupla (`real_T = double`) e simples
(`real32_T = float`). Elas são usadas em verificações de guarda em todo o
código gerado (ex.: "o resultado é finito?").

### 2.2 Cabeçalho associado (`rt_nonfinite.h`)

```cpp
// include/rt_nonfinite.h
extern real_T  rtNaN;       // double  NaN
extern real_T  rtInf;       // double +Inf
extern real_T  rtMinusInf;  // double -Inf
extern real32_T rtNaNF;     // float   NaN
extern real32_T rtInfF;     // float  +Inf
extern real32_T rtMinusInfF;// float  -Inf
```

> As declarações `extern` permitem que qualquer unidade de compilação que inclua
> `rt_nonfinite.h` acesse as constantes definidas em `rt_nonfinite.cpp`.

### 2.3 Implementação bloco a bloco

```cpp
// Bloco 1 — Includes
#include "rt_nonfinite.h"
#include <cmath>
#include <limits>
```
> - `rt_nonfinite.h`: necessário para que a unidade de compilação seja coerente
>   com as declarações `extern`.
> - `<cmath>` e `<limits>`: usados para `std::numeric_limits<T>::quiet_NaN()` e
>   `std::numeric_limits<T>::infinity()`.

```cpp
// Bloco 2 — Inicialização em precisão dupla
real_T rtNaN      = std::numeric_limits<real_T>::quiet_NaN();
real_T rtInf      = std::numeric_limits<real_T>::infinity();
real_T rtMinusInf = -std::numeric_limits<real_T>::infinity();
```
> `quiet_NaN()` retorna um NaN "silencioso" (não gera exceção de FP). Isso segue
> o padrão IEEE-754 e garante que verificações como `std::isnan(rtNaN)` retornem
> `true` sem efeitos colaterais.

```cpp
// Bloco 3 — Inicialização em precisão simples
real32_T rtNaNF      = std::numeric_limits<real32_T>::quiet_NaN();
real32_T rtInfF      = std::numeric_limits<real32_T>::infinity();
real32_T rtMinusInfF = -std::numeric_limits<real32_T>::infinity();
```
> Réplica do bloco 2 para `float`. Necessário porque parte do código gerado pelo
> MATLAB Coder opera em `single` (precisão simples).

### 2.4 Onde é usado

| Arquivo usuário | Uso |
|-----------------|-----|
| `mldivide.cpp` | Inclui `rt_nonfinite.h` (herança do codegen; não usa ativamente) |
| `minOrMax.cpp` | Inclui `rt_nonfinite.h` (idem) |
| `Drone_codegen.cpp` | Pode usar `rtNaN` em guards de resultado inválido |

---

## 3. `minOrMax.cpp` — Saturação de Vetores e Escalares

### 3.1 Papel

Fornece quatro funções de saturação usadas pelo controlador PID (`Drone_codegen`)
para limitar velocidades e acelerações desejadas a ±12 m/s. O limite ±12 m/s é
**hard-coded** diretamente no código gerado — ele reflete a parametrização original
do modelo MATLAB do controlador.

### 3.2 Cabeçalho associado

O arquivo `include/minOrMax.h` é **vazio** (placeholder gerado pelo MATLAB Coder);
as declarações funcionais estão embutidas dentro do namespace `coder::internal` e
são acessadas via inclusão de `minOrMax.h` em `Drone_codegen.cpp`.

### 3.3 Implementação bloco a bloco

#### Bloco 1 — Includes e namespace

```cpp
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include <cmath>

namespace coder {
namespace internal {
```
> - `rt_nonfinite.h`: herdado do template do MATLAB Coder; não é usado ativamente
>   neste arquivo mas mantido para consistência com o pipeline de codegen.
> - `<cmath>`: fornece `std::fmax` e `std::fmin`.
> - `namespace coder::internal`: isola as funções do namespace global, evitando
>   colisões com funções padrão de mesmo nome.

#### Bloco 2 — `maximum2(const double x[3], double ex[3])` — saturação inferior vetorial

```cpp
void maximum2(const double x[3], double ex[3])
{
  ex[0] = std::fmax(x[0], -12.0);
  ex[1] = std::fmax(x[1], -12.0);
  ex[2] = std::fmax(x[2], -12.0);
}
```

| Elemento | Descrição |
|----------|-----------|
| **Entrada** | `x[3]` — vetor de 3 doubles (ex.: velocidades desejadas XYZ) |
| **Saída** | `ex[3]` — resultado: cada componente é `max(x[i], -12.0)` |
| **Efeito** | Garante que nenhum componente seja **menor que −12 m/s** (satura pelo piso) |
| **`std::fmax`** | Lida corretamente com NaN: se `x[i]` for NaN, retorna `-12.0` |

> **Por que −12?** O modelo MATLAB original foi parametrizado com saturação de
> velocidade em ±12 m/s para o drone. O limite inferior (`−12`) é aplicado pela
> função `maximum2` (satura pelo *máximo* de x e −12, ou seja, rejeita valores
> abaixo de −12).

#### Bloco 3 — `maximum2(double x, double y)` — máximo escalar

```cpp
double maximum2(double x, double y)
{
  return std::fmax(x, y);
}
```

> Sobrecarga escalar. Usada quando o controlador calcula `max(valor, limite_inferior)`
> para um único double. Mesma semântica de NaN-safe que a versão vetorial.

#### Bloco 4 — `minimum2(const double x[3], double ex[3])` — saturação superior vetorial

```cpp
void minimum2(const double x[3], double ex[3])
{
  ex[0] = std::fmin(x[0], 12.0);
  ex[1] = std::fmin(x[1], 12.0);
  ex[2] = std::fmin(x[2], 12.0);
}
```

| Elemento | Descrição |
|----------|-----------|
| **Entrada** | `x[3]` — vetor de 3 doubles |
| **Saída** | `ex[3]` — resultado: cada componente é `min(x[i], 12.0)` |
| **Efeito** | Garante que nenhum componente seja **maior que +12 m/s** (satura pelo teto) |

> Ao aplicar `maximum2` seguido de `minimum2` no mesmo vetor, o controlador efetua
> uma **saturação simétrica ±12 m/s** equivalente a `clamp(x, -12, 12)`.

#### Bloco 5 — `minimum2(double x, double y)` — mínimo escalar

```cpp
double minimum2(double x, double y)
{
  return std::fmin(x, y);
}
```

> Sobrecarga escalar para a saturação superior. Usada individualmente pelo
> `Drone_codegen` quando opera em componentes isoladas (ex.: `zdot_des`).

### 3.4 Padrão de uso em `Drone_codegen.cpp`

```cpp
// Exemplo: saturar vetor de velocidade desejada [-12, +12] m/s
double vel_raw[3] = { vx_des, vy_des, vz_des };
double vel_clamp[3];
coder::internal::maximum2(vel_raw, vel_clamp);   // aplica piso −12
coder::internal::minimum2(vel_clamp, vel_clamp); // aplica teto +12
// vel_clamp[] agora está em [-12, 12] m/s em cada eixo
```

---

## 4. `mldivide.cpp` — Solver de Sistemas Lineares por Eliminação Gaussiana com Pivotamento Parcial

### 4.1 Papel

Implementa o operador MATLAB `A \ B` (divisão à esquerda — solução do sistema
`A·X = B`) para matrizes de tamanho fixo (4×4, 6×6 e 3×3). É usado por
`TrajectoryPlanner_codegen` para calcular os **coeficientes polinomiais** do
planner de trajetória — o "ajuste" de um polinômio de 5ª ordem que passa pelos
waypoints dados.

### 4.2 Algoritmo: Eliminação Gaussiana com Pivotamento Parcial (LU)

O algoritmo decompõe `A` como `P·A = L·U` onde:
- **P** é a matriz de permutação (pivotamento de linhas)
- **L** é triangular inferior com diagonal 1 (multipliers)
- **U** é triangular superior

A solução `X` é então encontrada em dois passos:
1. **Substituição direta** (`L·Y = P·B`): resolve de cima para baixo.
2. **Substituição regressiva** (`U·X = Y`): resolve de baixo para cima.

### 4.3 Sobrecarga `b_mldivide` (4×4) — bloco a bloco

```cpp
void b_mldivide(const double A[16], double B[4])
```

> **Entrada:** `A[16]` — matriz 4×4 armazenada em **ordem de coluna** (column-major,
> como no MATLAB); `B[4]` — vetor de termos independentes.  
> **Saída:** `B[4]` (in-place) — vetor solução X tal que `A·X = B`.

#### Bloco 1 — Cópia da matriz e inicialização do vetor de pivôs

```cpp
double b_A[16];
signed char ipiv[4];
std::copy(&A[0], &A[16], &b_A[0]);  // cópia para não destruir A
ipiv[0] = 1; ipiv[1] = 2; ipiv[2] = 3; ipiv[3] = 4;  // identidade inicial
```
> `b_A` é a cópia de trabalho (será fatorada in-place em L e U).  
> `ipiv[j]` registra para qual linha a linha `j` foi trocada durante o
> pivotamento; começa em `{1,2,3,4}` (nenhuma troca).

#### Bloco 2 — Loop externo: fatoração LU coluna por coluna (j = 0..2)

```cpp
for (int j{0}; j < 3; j++) {
  int b_tmp = j * 5;   // índice do elemento diagonal: b_A[j][j] = b_A[j*4 + j]
                        // Nota: b_tmp = j*5 porque j*4 + j = j*(4+1) = j*5
```
> O índice `b_tmp = j*5` acessa o elemento diagonal `A[j,j]` em order column-major:
> coluna `j`, linha `j` → índice `j*4 + j = 5j`.

#### Bloco 3 — Busca do pivô máximo na coluna `j` (linhas j..3)

```cpp
  a = 0;
  smax = std::abs(b_A[b_tmp]);
  for (int k{2}; k <= jA; k++) {
    double s = std::abs(b_A[(b_tmp + k) - 1]);
    if (s > smax) { a = k - 1; smax = s; }
  }
```
> Percorre os elementos abaixo da diagonal na coluna `j` e encontra o de maior
> valor absoluto. `a` guarda o *offset* relativo do pivô (0 = diagonal já é o
> maior; `a > 0` = troca necessária).  
> **Por que pivotamento parcial?** Para evitar divisão por valores próximos de
> zero (instabilidade numérica) e garantir que `|A[j,j]|` seja o maior da coluna.

#### Bloco 4 — Troca de linhas (row swap)

```cpp
  if (b_A[b_tmp + a] != 0.0) {
    if (a != 0) {
      jA = j + a;
      ipiv[j] = static_cast<signed char>(jA + 1);  // registra a troca
      // Troca a linha j com a linha jA em TODAS as 4 colunas
      smax = b_A[j];       b_A[j]    = b_A[jA];    b_A[jA]    = smax;
      smax = b_A[j + 4];   b_A[j+4]  = b_A[jA+4];  b_A[jA+4]  = smax;
      smax = b_A[j + 8];   b_A[j+8]  = b_A[jA+8];  b_A[jA+8]  = smax;
      smax = b_A[j + 12];  b_A[j+12] = b_A[jA+12]; b_A[jA+12] = smax;
    }
```
> Troca explícita dos 4 elementos de cada coluna (a matriz 4×4 tem 4 colunas).
> A troca é "desenrolada" manualmente pelo MATLAB Coder (sem loop) para melhor
> desempenho em matrizes de tamanho fixo.

#### Bloco 5 — Divisão pelo pivô (computação dos multiplicadores L)

```cpp
    i = (b_tmp - j) + 4;
    for (a = jp1j; a <= i; a++) {
      b_A[a - 1] /= b_A[b_tmp];
    }
```
> Divide todos os elementos **abaixo** do pivô na coluna `j` pelo valor do pivô.
> O resultado é o vetor de multiplicadores de Gauss (subdiagonal de L), que será
> usado na eliminação da linha do passo seguinte.

#### Bloco 6 — Eliminação: atualização do submatriz

```cpp
    jA = b_tmp;
    for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
      smax = b_A[(b_tmp + (jp1j << 2)) + 4];  // elemento acima da diagonal na linha seguinte
      if (smax != 0.0) {
        i = jA + 6; a = (jA - j) + 8;
        for (int k{i}; k <= a; k++) {
          b_A[k - 1] += b_A[((b_tmp + k) - jA) - 5] * -smax;
        }
      }
      jA += 4;
    }
```
> Para cada coluna `jp1j` à direita da coluna atual, subtrai `multiplicador × linha_j`
> da linha alvo. Isso é a **eliminação gaussiana** propriamente dita: zera os
> elementos abaixo do pivô em todas as colunas restantes.  
> A guarda `if (smax != 0.0)` evita operações desnecessárias quando o elemento
> já é zero.

#### Bloco 7 — Aplicação de pivotamento ao vetor B

```cpp
    i1 = ipiv[j];
    if (i1 != j + 1) {
      smax   = B[j];
      B[j]   = B[i1 - 1];
      B[i1 - 1] = smax;
    }
```
> Aplica ao vetor `B` as mesmas trocas de linha que foram aplicadas a `A`.
> Equivalente a multiplicar `B` pela matriz de permutação `P`.

#### Bloco 8 — Substituição direta (`L·Y = P·B`)

```cpp
for (int k{0}; k < 4; k++) {
  jA = k << 2;           // jA = 4*k = índice da coluna k
  if (B[k] != 0.0) {
    for (a = k + 2; a < 5; a++) {
      B[a - 1] -= B[k] * b_A[(a + jA) - 1];  // elimina L abaixo da diagonal
    }
  }
}
```
> Resolve o sistema triangular inferior `L·Y = P·B`.  
> Como a diagonal de L é 1 (por construção), não é necessário dividir por ela.
> `B` é atualizado in-place: após este loop, `B` contém o vetor intermediário `Y`.

#### Bloco 9 — Substituição regressiva (`U·X = Y`)

```cpp
for (int k{3}; k >= 0; k--) {
  jA = k << 2;
  smax = B[k];
  if (smax != 0.0) {
    smax /= b_A[k + jA];   // divide pelo elemento diagonal de U
    B[k] = smax;
    for (a = 0; a < k; a++) {
      B[a] -= B[k] * b_A[a + jA];  // elimina U acima da diagonal
    }
  }
}
```
> Resolve o sistema triangular superior `U·X = Y`.  
> Divide `B[k]` pelo elemento diagonal `U[k,k]` para obter `X[k]`, depois
> elimina a contribuição de `X[k]` nas linhas acima. Iteração de `k=3` até `k=0`.
> Ao final, `B[4]` contém a solução `X` do sistema original `A·X = B`.

### 4.4 Sobrecarga `mldivide` (6×6) — visão geral

```cpp
void mldivide(const double A[36], double B[6])
```

> Mesma estrutura do algoritmo 4×4, mas para matriz 6×6 (sistema de 6 equações).
> Diferença principal: a troca de linhas usa um loop de 6 colunas em vez de 4
> trocas explícitas:

```cpp
for (int k{0}; k < 6; k++) {
  a = j + k * 6;       // índice linha j, coluna k
  A_tmp = jA + k * 6;  // índice linha jA, coluna k
  smax = b_A[a];       b_A[a] = b_A[A_tmp]; b_A[A_tmp] = smax;
}
```

> **Uso em `TrajectoryPlanner_codegen`:** resolve o sistema linear `M·c = b` onde
> `M` é a matriz de Vandermonde 6×6 montada a partir dos tempos dos waypoints e
> `b` contém as condições de contorno do polinômio de 5ª ordem (posição, velocidade
> e aceleração nos extremos do segmento). A solução `c` são os 6 coeficientes do
> polinômio que define a trajetória suave naquele segmento.

### 4.5 Sobrecarga `mldivide` (3×3) — visão geral

```cpp
void mldivide(const double A[9], const double B[3], double Y[3])
```

> Para sistemas 3×3, usa pivotamento seletivo (escolha de pivô entre as 3 linhas
> da primeira coluna) seguido de eliminação gaussiana simplificada. **Não usa o
> array `ipiv`** — os índices de linha são rastreados diretamente nas variáveis
> `r1`, `r2`, `r3`:

```cpp
r1 = 0; r2 = 1; r3 = 2;
maxval = std::abs(A[0]);
a21    = std::abs(A[1]);
if (a21 > maxval)      { maxval = a21; r1 = 1; r2 = 0; }
if (std::abs(A[2]) > maxval) { r1 = 2; r2 = 1; r3 = 0; }
```
> Compara os 3 primeiros elementos da primeira coluna e reorganiza `r1,r2,r3`
> para que `r1` aponte para a linha de maior valor absoluto (o pivô).

---

## 5. `cos.cpp` — Stub Vazio (Compatibilidade de Codegen)

### 5.1 Papel

O arquivo `src/cos.cpp` é um **stub vazio** gerado pelo MATLAB Coder para declarar
a função `coder::b_cos(double &x)`. Em algumas versões do modelo MATLAB, a função
cosseno do controlador é emitida como uma chamada a `b_cos`; quando o compilador
MATLAB Coder detecta que `std::cos` pode ser substituída por um wrapper inlinável,
gera este arquivo como placeholder.

### 5.2 Cabeçalho (`cos.h`)

```cpp
// include/cos.h
namespace coder {
  void b_cos(double &x);
}
```
> Declara `b_cos` como wrapper de `std::cos`. A função **modifica `x` in-place**
> (resultado substitui a entrada). Esta assinatura é diferente da `std::cos` padrão
> (`double cos(double)`) para se adequar ao padrão de chamada do código gerado pelo
> MATLAB Coder.

### 5.3 Implementação

O arquivo `src/cos.cpp` está **vazio** — nenhum corpo de função é definido. Isso
significa que `b_cos` nunca é chamada em tempo de execução no build atual: o
MATLAB Coder otimizou as chamadas de cosseno para chamadas diretas a `std::cos`
dentro de `Drone_codegen.cpp` e/ou `TrajectoryPlanner_codegen.cpp`, tornando o
wrapper desnecessário.

> **Se `b_cos` fosse necessária:** o arquivo seria completado com:
> ```cpp
> #include "cos.h"
> #include <cmath>
> namespace coder {
>   void b_cos(double &x) { x = std::cos(x); }
> }
> ```

### 5.4 Relação com a FSM

Nenhuma. O arquivo existe apenas para que o linker não reclame de uma referência
indefinida a `coder::b_cos` caso o cabeçalho `cos.h` seja incluído em alguma
unidade de compilação.

---

## 6. `sqrt.cpp` — Stub Vazio (Compatibilidade de Codegen)

### 6.1 Papel

Idêntico ao `cos.cpp` — arquivo **stub vazio** gerado pelo MATLAB Coder. O cabeçalho
`include/sqrt.h` também está vazio (sem declarações visíveis), indicando que a
função raiz quadrada é acessada diretamente via `<cmath>` no código gerado, sem
necessitar de wrapper.

### 6.2 Por que o arquivo existe no build?

O MATLAB Coder inclui estes stubs no manifesto de arquivos fonte por **consistência
de template**: o mesmo `CMakeLists.txt` funciona tanto para versões do modelo onde
`b_cos`/`b_sqrt` são necessárias (chamadas indiretas via wrapper) quanto para
versões onde o compilador as inlina. Desta forma, não é necessário modificar o
sistema de build ao re-gerar o código a partir de um modelo MATLAB diferente.

### 6.3 Relação com a FSM

Nenhuma direta. `std::sqrt` é chamada diretamente nos cálculos de norma
(`dist_xy = std::sqrt(dx*dx + dy*dy)`) dentro de `fsm_trajectory.cpp`, sem
passar por este stub.

---

## 7. Diagrama de Dependências Completo (Codegen)

```
TrajectoryPlanner_codegen.cpp
  │ inclui
  ├─► TrajectoryPlanner_codegen.h
  ├─► mldivide.h        ──► mldivide.cpp   (solver LU para coefs polinomiais)
  ├─► rt_nonfinite.h    ──► rt_nonfinite.cpp (constantes NaN/Inf)
  └─► coder_array.h     (template de array dinâmico, header-only)

Drone_codegen.cpp
  │ inclui
  ├─► Drone_codegen.h
  ├─► minOrMax.h        ──► minOrMax.cpp   (saturação ±12 m/s)
  ├─► mldivide.h        ──► mldivide.cpp   (usado para cálculo interno do PID)
  ├─► rt_nonfinite.h    ──► rt_nonfinite.cpp
  ├─► cos.h             ──► cos.cpp        (stub vazio)
  └─► sqrt.h            ──► sqrt.cpp       (stub vazio)
```

---

## 8. Relação com `DroneControllerCompleto` e FSM

| Helper | Quando é ativo | Estado FSM | Efeito observável |
|--------|---------------|------------|-------------------|
| `rt_nonfinite` | Inicialização do processo | — | Constantes globais disponíveis para todo o codegen |
| `minOrMax` | A cada iteração do controlador | Estado 3 (trajetória) | Velocidades desejadas `Vd` e `zdot_des` são saturadas em ±12 m/s |
| `mldivide` | `planner_.init()` (uma vez por trajetória) | Estado 3 (init) | Calcula os 6 coeficientes por segmento do polinômio de trajetória |
| `cos` (stub) | Nunca em runtime | — | Sem efeito |
| `sqrt` (stub) | Nunca em runtime | — | Sem efeito |

> **Nota de desempenho:** `mldivide` é chamado apenas **uma vez por trajetória**
> (em `initialize_trajectory()`, via `planner_.init()`), não a cada iteração de
> 10 ms. `minOrMax` é chamado a cada ciclo do controlador (100 Hz) mas com
> operações O(1) (3 chamadas a `std::fmax`/`std::fmin`), sem impacto de latência.
