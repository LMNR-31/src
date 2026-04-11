# 14 — Sistema de Fila de Comandos (`command_queue.hpp` + `command_queue.cpp`)

> **Arquivos:** `include/my_drone_controller/command_queue.hpp` e `src/command_queue.cpp`

Este módulo implementa um **sistema de rastreabilidade de comandos** com ID único, histórico persistente e detecção de timeout. Funciona como um livro de registro de auditoria de cada ação executada pelo drone.

---

## 1. Tipos e Enums (`command_queue.hpp`)

### `CommandType` — tipo de comando

```cpp
enum class CommandType {
  ARM,               // Solicitar ARM ao FCU
  DISARM,            // Solicitar DISARM ao FCU
  SET_MODE_OFFBOARD, // Solicitar modo OFFBOARD ao FCU
  TAKEOFF,           // Decolagem (stream + OFFBOARD + ARM + subida)
  HOVER,             // Manutenção de posição (Estado 2)
  TRAJECTORY,        // Execução de trajetória (Estado 3)
  LAND               // Pouso (Estado 4)
};
```

Cada valor representa uma "fase" da missão. Quando um comando é enfileirado, o sistema registra **qual fase foi iniciada** e **quando**.

### `CommandStatus` — ciclo de vida de um comando

```cpp
enum class CommandStatus {
  PENDING,    // Enviado ao FCU, aguardando confirmação.
  CONFIRMED,  // FCU confirmou sucesso (ex.: drone atingiu altitude, pousou).
  FAILED,     // FCU ou controlador reportou falha.
  TIMEOUT     // Nenhuma confirmação dentro do janela de timeout.
};
```

O ciclo de vida normal é:
```
enqueue() → PENDING → confirm(true) → CONFIRMED
                    → confirm(false) → FAILED
                    → check_timeouts() → TIMEOUT
```

### `Command` — estrutura de um comando individual

```cpp
struct Command {
  uint64_t id{0};                          // ID único auto-incrementado (começa em 1)
  CommandType type{CommandType::ARM};       // Qual tipo de ação
  CommandStatus status{CommandStatus::PENDING}; // Estado atual
  std::chrono::system_clock::time_point timestamp{};    // Instante de criação
  std::chrono::system_clock::time_point confirm_time{}; // Instante de confirmação
  std::map<std::string, std::string> data; // Dados extras (ex.: {"x":"1.0","z":"2.5"})

  std::string type_str() const { ... }   // "TAKEOFF", "HOVER", etc.
  std::string status_str() const { ... } // "PENDENTE", "CONFIRMADO", etc.
};
```

O campo `data` é um mapa de strings livre — cada tipo de comando armazena o que for relevante:

| Tipo | Exemplo de `data` |
|---|---|
| `TAKEOFF` | `{"x":"5.0", "y":"3.0", "z":"2.5"}` |
| `HOVER` | `{"x":"5.0", "y":"3.0", "z":"2.5"}` |
| `TRAJECTORY` | `{"waypoints":"5"}` |
| `LAND` | `{"z":"-0.1"}` |

---

## 2. Classe `CommandQueue`

### Atributos privados

```cpp
mutable std::mutex mutex_;      // protege pending_ e history_ contra acesso concorrente
                                // 'mutable' permite lock em métodos const como get_history()
uint64_t next_id_{1};           // contador de IDs; incrementado a cada enqueue()
std::map<uint64_t, Command> pending_;  // comandos ainda não confirmados (vivos)
std::vector<Command> history_;         // todos os comandos já criados (auditoria completa)
```

---

## 3. Métodos — `command_queue.cpp`

### 3.1 Construtor e Destrutor

```cpp
CommandQueue::CommandQueue()
: next_id_(1)   // primeiro ID será 1 (IDs são monotonicamente crescentes)
{}

CommandQueue::~CommandQueue()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Limpa os mapas de forma thread-safe ao destruir o objeto.
  // Impede que callbacks em outros threads acessem memória liberada.
  pending_.clear();
  history_.clear();
}
```

### 3.2 `enqueue()` — adicionar comando

```cpp
uint64_t CommandQueue::enqueue(CommandType type,
                               const std::map<std::string, std::string> & data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // ↑ Adquire o mutex: garante que nenhuma outra thread modifica pending_/history_
  //   enquanto estamos inserindo.

  Command cmd;
  cmd.id        = next_id_++;   // atribui ID único e incrementa o contador
  cmd.type      = type;         // ex.: CommandType::TAKEOFF
  cmd.status    = CommandStatus::PENDING;  // todo novo comando começa PENDENTE
  cmd.timestamp = std::chrono::system_clock::now();  // registra instante de criação
  cmd.data      = data;         // cópia dos metadados (ex.: {"z":"2.5"})

  pending_[cmd.id] = cmd;   // insere em pending_ (commandos ativos)
  history_.push_back(cmd);  // insere em history_ (log imutável)

  return cmd.id;  // retorna ID para que o chamador possa confirmar depois
}
```

**Uso típico:**
```cpp
// Em handle_single_takeoff_waypoint_command():
takeoff_cmd_id_ = cmd_queue_.enqueue(
  CommandType::TAKEOFF,
  {{"x", std::to_string(x)},
   {"y", std::to_string(y)},
   {"z", std::to_string(config_.hover_altitude)}});
// Retorna std::optional<uint64_t> com o ID para confirmação posterior.
```

### 3.3 `confirm()` — confirmar ou reprovar

```cpp
bool CommandQueue::confirm(uint64_t id, bool success)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Busca o comando em pending_:
  auto it = pending_.find(id);
  if (it == pending_.end()) {
    return false;  // ID não existe em pending_ — já confirmado ou inválido
  }

  // Determina o novo status (CONFIRMED ou FAILED):
  CommandStatus new_status = success ? CommandStatus::CONFIRMED : CommandStatus::FAILED;
  it->second.status       = new_status;
  it->second.confirm_time = std::chrono::system_clock::now();  // registra instante

  // Sincroniza history_ com o novo status (histórico deve refletir o resultado real):
  for (auto & h : history_) {
    if (h.id == id) {
      h.status       = new_status;
      h.confirm_time = it->second.confirm_time;
      break;
    }
  }

  // Remove de pending_ — o comando não está mais "esperando confirmação":
  pending_.erase(it);
  return true;  // confirmação bem-sucedida
}
```

**Uso típico:**
```cpp
// Em finalize_takeoff_on_altitude_reached():
cmd_queue_.confirm(*takeoff_cmd_id_, true);  // TAKEOFF → CONFIRMED
takeoff_cmd_id_.reset();                     // libera o optional

// Em detect_and_handle_landing_in_trajectory():
cmd_queue_.confirm(*trajectory_cmd_id_, false);  // TRAJECTORY → FAILED (interrompida)
```

### 3.4 `check_timeouts()` — verificar expirados

```cpp
std::vector<uint64_t> CommandQueue::check_timeouts(double timeout_seconds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint64_t> timed_out;  // IDs que expiraram nesta chamada
  auto now = std::chrono::system_clock::now();

  for (auto it = pending_.begin(); it != pending_.end(); ) {
    double elapsed = std::chrono::duration<double>(now - it->second.timestamp).count();
    // elapsed = tempo em segundos desde a criação do comando

    if (elapsed > timeout_seconds) {
      // Marca como TIMEOUT no próprio pending_ (antes de remover):
      it->second.status = CommandStatus::TIMEOUT;

      // Sincroniza history_ (sob o mesmo lock — sem corrida de dados):
      for (auto & h : history_) {
        if (h.id == it->second.id) {
          h.status = CommandStatus::TIMEOUT;
          break;
        }
      }

      timed_out.push_back(it->second.id);
      it = pending_.erase(it);  // remove de pending_ e avança o iterador
    } else {
      ++it;  // não expirou, avança normalmente
    }
  }
  return timed_out;  // lista de IDs que expiraram nesta verificação
}
```

Chamado periodicamente (a cada ~10 s) pelo `control_loop()` via `check_command_timeouts()`:

```cpp
// No control_loop(), a cada 1000 ciclos ≈ 10 s:
auto timed_out = cmd_queue_.check_timeouts(config_.command_timeout);
for (auto id : timed_out) {
  RCLCPP_WARN(this->get_logger(),
    "⚠️ Comando ID=%lu expirou sem confirmação!", id);
}
```

### 3.5 `cancel_all_pending()` — cancelar tudo

```cpp
void CommandQueue::cancel_all_pending()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto now = std::chrono::system_clock::now();

  for (auto & [id, cmd] : pending_) {
    // Marca como FAILED com timestamp atual:
    cmd.status       = CommandStatus::FAILED;
    cmd.confirm_time = now;

    // Sincroniza history_:
    for (auto & h : history_) {
      if (h.id == id) {
        h.status       = CommandStatus::FAILED;
        h.confirm_time = now;
        break;
      }
    }
  }
  pending_.clear();  // remove todos os pendentes
}
```

Chamado no início de um novo ciclo de missão para limpar comandos antigos que nunca foram confirmados (ex.: trajetória que foi interrompida mid-flight).

### 3.6 `save_log()` — salvar histórico em arquivo

```cpp
void CommandQueue::save_log(const std::string & filename) const
{
  std::lock_guard<std::mutex> lock(mutex_);  // mutable lock em const

  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "[CommandQueue] ERRO: Não foi possível abrir: " << filename << "\n";
    return;
  }

  file << "=== HISTORICO DE COMANDOS DO DRONE ===\n";
  file << "Total: " << history_.size() << " comandos\n\n";

  for (const auto & cmd : history_) {
    // Converte timestamp para string legível (ex.: "2025-04-11 16:30:00"):
    std::time_t t = std::chrono::system_clock::to_time_t(cmd.timestamp);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);  // thread-safe em POSIX

    file << std::put_time(&tm_buf, "[%Y-%m-%d %H:%M:%S]")
         << " ID=" << std::setw(4) << std::right << cmd.id
         << " | TIPO="   << std::setw(18) << std::left << cmd.type_str()
         << " | STATUS=" << std::setw(10) << std::left << cmd.status_str();

    // Dados extras (ex.: "{x=5.00, y=3.00, z=2.50}"):
    if (!cmd.data.empty()) {
      file << " | DADOS={";
      bool first = true;
      for (const auto & kv : cmd.data) {
        if (!first) { file << ", "; }
        file << kv.first << "=" << kv.second;
        first = false;
      }
      file << "}";
    }

    // Tempo de execução (apenas para comandos confirmados ou falhos):
    if (cmd.status == CommandStatus::CONFIRMED || cmd.status == CommandStatus::FAILED) {
      double elapsed = std::chrono::duration<double>(
        cmd.confirm_time - cmd.timestamp).count();
      file << " | TEMPO=" << std::fixed << std::setprecision(2) << elapsed << "s";
    }
    file << "\n";
  }
  file.close();
}
```

**Exemplo de arquivo `/tmp/drone_commands.log`:**
```
=== HISTORICO DE COMANDOS DO DRONE ===
Total: 5 comandos

[2025-04-11 16:30:01] ID=   1 | TIPO=TAKEOFF            | STATUS=CONFIRMADO | DADOS={x=5.00, y=3.00, z=2.50} | TEMPO=8.23s
[2025-04-11 16:30:09] ID=   2 | TIPO=HOVER              | STATUS=CONFIRMADO | DADOS={x=5.00, y=3.00, z=2.50} | TEMPO=5.12s
[2025-04-11 16:30:14] ID=   3 | TIPO=TRAJECTORY         | STATUS=CONFIRMADO | DADOS={waypoints=4}             | TEMPO=32.45s
[2025-04-11 16:30:46] ID=   4 | TIPO=LAND               | STATUS=CONFIRMADO | DADOS={z=-0.10}                 | TEMPO=6.88s
[2025-04-11 16:30:53] ID=   5 | TIPO=DISARM             | STATUS=CONFIRMADO | DADOS={}                        | TEMPO=0.31s
```

---

## 4. Thread-Safety

Todo método que acessa `pending_` ou `history_` usa `std::lock_guard<std::mutex>`:
- O `control_loop()` chama `enqueue()` e `confirm()` a 100 Hz.
- Callbacks de subscribers (rodando em threads do executor ROS 2) não acessam diretamente a fila, mas `control_loop()` pode rodar enquanto um callback atualiza variáveis de estado.
- O mutex garante que `pending_` e `history_` nunca são lidos e escritos simultaneamente.

---

## 5. Diagrama de vida de um comando

```
enqueue(TAKEOFF, {x,y,z})
  └─ Command{id=1, status=PENDING, timestamp=T0}
        ↓ inserido em pending_[1] e history_[0]

 ... drone decola durante N segundos ...

confirm(1, true)           (quando altitude atingida)
  └─ pending_[1].status = CONFIRMED, confirm_time = T1
  └─ history_[0].status = CONFIRMED  (sincronizado)
  └─ pending_.erase(1)   (saiu de pending_)

 --- OU ---

check_timeouts(30.0)       (se não confirmado em 30 s)
  └─ pending_[1].status = TIMEOUT
  └─ history_[0].status = TIMEOUT
  └─ pending_.erase(1)

save_log("/tmp/drone_commands.log")
  └─ history_[0] → "[2025-...] ID=1 | TIPO=TAKEOFF | STATUS=CONFIRMADO | TEMPO=8.23s"
```
