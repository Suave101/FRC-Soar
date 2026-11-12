#include "soarbridge/SoarHardwareInterface.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <utility>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

#include "sml_Client.h"

namespace soarbridge {

namespace {

using Clock = std::chrono::steady_clock;

// Command parameters nested deeper than this are rejected (also stops cycles).
constexpr int kMaxParamDepth = 4;

const char* ModeName(MatchMode mode) {
  switch (mode) {
    case MatchMode::Autonomous: return "autonomous";
    case MatchMode::Teleop:     return "teleop";
    case MatchMode::Test:       return "test";
    case MatchMode::Disabled:   break;
  }
  return "disabled";
}

const char* AllianceName(Alliance alliance) {
  return alliance == Alliance::Red ? "red" : "blue";
}

// Dot-separated segments, each a letter followed by letters, digits, '-' or
// '_'. With `single`, exactly one segment (a command name).
bool ValidPath(const std::string& path, bool single = false) {
  if (path.empty()) return false;
  bool segmentStart = true;
  for (char ch : path) {
    unsigned char u = static_cast<unsigned char>(ch);
    if (ch == '.') {
      if (single || segmentStart) return false;
      segmentStart = true;
    } else if (segmentStart) {
      if (!std::isalpha(u)) return false;
      segmentStart = false;
    } else if (!std::isalnum(u) && ch != '-' && ch != '_') {
      return false;
    }
  }
  return !segmentStart;
}

// True if `key` is `path` or lies under it ("a.b" is under "a", "ab" is not).
bool IsAtOrUnder(const std::string& key, const std::string& path) {
  return key.compare(0, path.size(), path) == 0 &&
         (key.size() == path.size() || key[path.size()] == '.');
}

// One input value on its way to the input-link.
struct InputChange {
  std::string path;
  Value value;
};

// Change buffers are reused rather than cleared: assigning into an element
// whose strings already have capacity does not allocate, so the robot thread
// does no heap allocation in steady state (only for new paths and removals).
void PutChange(std::vector<InputChange>* buf, size_t* count, const std::string& path, const Value& v) {
  if (*count < buf->size()) {
    InputChange& c = (*buf)[*count];
    c.path.assign(path);
    c.value = v;
  } else {
    buf->push_back(InputChange{path, v});
  }
  ++*count;
}

Value ReadValue(sml::WMElement* wme) {
  if (sml::IntElement* i = wme->ConvertToIntElement()) return Value::FromInt(i->GetValue());
  if (sml::FloatElement* f = wme->ConvertToFloatElement()) return Value::FromFloat(f->GetValue());
  const char* text = wme->GetValueAsString();
  return Value::FromString(text ? text : "");
}

// Flattens the children of a command identifier into dotted keys.
bool ReadParams(sml::Identifier* id, const std::string& prefix, int depth,
                const std::string& name, std::map<std::string, Value>* out, std::string* why) {
  if (depth > kMaxParamDepth) {
    *why = "^" + name + " parameters are nested more than " + std::to_string(kMaxParamDepth) + " levels deep";
    return false;
  }
  for (auto it = id->GetChildrenBegin(); it != id->GetChildrenEnd(); ++it) {
    sml::WMElement* child = *it;
    const char* attr = child->GetAttribute();
    if (attr == nullptr) continue;
    if (depth == 0 && !std::strcmp(attr, "status")) continue;  // written by the bridge
    std::string key = prefix + attr;
    if (child->IsIdentifier()) {
      if (!ReadParams(child->ConvertToIdentifier(), key + ".", depth + 1, name, out, why)) return false;
      continue;
    }
    if (!out->emplace(key, ReadValue(child)).second) {
      *why = "^" + name + " has more than one ^" + key;
      return false;
    }
  }
  return true;
}

bool Number(const Command& c, const char* key, double* out) {
  auto it = c.params.find(key);
  return it != c.params.end() && it->second.AsNumber(out);
}

// Fills the typed fields of a built-in command. Returns false with `why` set
// if a required parameter is missing or malformed. Not built-in: Custom.
bool FillBuiltIn(Command* c, bool* builtIn, std::string* why) {
  *builtIn = true;
  const std::string& n = c->name;
  bool ok;
  if (n == "move") {
    c->type = CommandType::Move;
    ok = Number(*c, "vx", &c->vx) && Number(*c, "vy", &c->vy) && Number(*c, "omega", &c->omega);
    if (!ok) *why = "^move needs numeric ^vx, ^vy and ^omega";
  } else if (n == "turn-to") {
    c->type = CommandType::TurnTo;
    ok = Number(*c, "heading", &c->heading);
    if (!ok) *why = "^turn-to needs a numeric ^heading";
  } else if (n == "set-elevator") {
    c->type = CommandType::SetElevator;
    ok = Number(*c, "height", &c->height);
    if (!ok) *why = "^set-elevator needs a numeric ^height";
  } else if (n == "run-intake") {
    c->type = CommandType::RunIntake;
    ok = Number(*c, "speed", &c->speed);
    if (!ok) *why = "^run-intake needs a numeric ^speed";
  } else if (n == "shoot") {
    c->type = CommandType::Shoot;
    auto it = c->params.find("execute");
    ok = it != c->params.end() && it->second.AsBool(&c->execute);
    if (!ok) *why = "^shoot needs ^execute true or false";
  } else {
    c->type = CommandType::Custom;
    *builtIn = false;
    ok = true;
  }
  return ok;
}

struct Completion {
  long long id;
  bool success;
};

}  // namespace

struct SoarHardwareInterface::Impl {
  explicit Impl(const BridgeConfig& c) : config(c) {}

  const BridgeConfig config;
  std::thread thread;

  // --- Robot side, guarded by inputMutex (never touched by the Soar thread).
  // Lock order: inputMutex before mutex.
  struct Entry {
    Value value;
    bool dirty = false;   // changed since the last SendInputs()
  };
  using Entries = std::map<std::string, Entry>;
  std::mutex inputMutex;
  Entries current;                       // every live input value, by path
  std::vector<Entries::iterator> dirty;  // entries changed since the last SendInputs()
  std::vector<std::string> removed;      // RemoveInput() paths since the last SendInputs()
  // Robot thread only (see OnCommand).
  std::map<std::string, CommandCallback> handlers;

  // Everything below is guarded by `mutex`. Neither thread holds it for more
  // than a small copy, a merge of one loop's changes, or a container swap.
  mutable std::mutex mutex;
  std::condition_variable wake;      // robot -> Soar: new input, completion, stop
  std::condition_variable started;   // Soar -> Start(): init finished
  enum class State { Stopped, Starting, Running, Failed } state = State::Stopped;
  bool stopRequested = false;
  std::string lastError;
  std::string lastWarning;

  // Published changes Soar has not applied yet. Applied as: every removal (in
  // order), then every value (in order). SendInputs() keeps that equivalent
  // to the order the calls were made by dropping values a later removal covers.
  std::vector<std::string> pendingRemovals;
  std::vector<InputChange> pendingValues;
  size_t pendingCount = 0;           // used elements of pendingValues
  bool inputsFresh = false;
  Clock::time_point inputsTime;

  std::set<std::string> accepted;    // AcceptCommand() names
  long long acceptedVersion = 0;

  std::vector<Command> commands;         // Soar -> robot
  std::vector<Completion> completions;   // robot -> Soar
  BridgeStats stats;

  // --- Soar thread only ---
  sml::Kernel* kernel = nullptr;
  sml::Agent* agent = nullptr;
  struct Slot {
    sml::WMElement* wme;
    bool isIdentifier;
    ValueType type;
  };
  std::map<std::string, Slot> wmes;       // input-link WMEs by path
  std::set<std::string> acceptedCopy;
  long long acceptedSeen = -1;

  void ThreadMain();
  bool InitSoar(std::string* error);
  void ApplyValue(const std::string& path, const Value& v);
  void DestroySubtree(const std::string& path);
  sml::Identifier* EnsureIdentifier(const std::string& path);
  bool MarkStatus(long long id, bool success);
  int ApplyCompletions(const std::vector<Completion>& done);
  void CollectCommands(std::vector<Command>* out, int* rejected, int* constants, std::string* warning);
  bool ParseCommand(sml::Identifier* cmd, Command* out, std::string* why);
  void ShutdownSoar();

  // Robot side; caller holds inputMutex and has validated the path.
  bool StageValue(const std::string& path, const Value& value);
  void MarkDirty(Entries::iterator it);
  // Caller holds inputMutex and mutex.
  void QueueAllValues();
  bool Refuse(const std::string& why);
  bool SetPath(const std::string& path, Value value);
  int Dispatch(CommandHandler* handler);
};

// ---------------------------------------------------------------------------
// Soar thread

bool SoarHardwareInterface::Impl::InitSoar(std::string* error) {
  int port = config.debuggerPort > 0 ? config.debuggerPort : sml::Kernel::kSuppressListener;
  kernel = sml::Kernel::CreateKernelInCurrentThread(true, port);
  if (kernel == nullptr || kernel->HadError()) {
    *error = std::string("Soar kernel creation failed: ") +
             (kernel ? kernel->GetLastErrorDescription() : "null kernel");
    return false;
  }
  agent = kernel->CreateAgent(config.agentName.c_str());
  if (agent == nullptr || kernel->HadError()) {
    *error = std::string("Soar agent creation failed: ") + kernel->GetLastErrorDescription();
    return false;
  }
  // Tracing costs time on the hot path and nobody is reading it.
  agent->ExecuteCommandLine("trace 0");
  // Re-sending an unchanged value must not retract and re-add the WME.
  agent->SetBlinkIfNoChange(false);
  // CollectCommands relies on IsJustAdded(), which ClearOutputLinkChanges()
  // only resets for tracked changes. SML turns tracking on lazily, at the
  // first GetOutputLink(); output from the very first run would then stay
  // "just added" for ever and be delivered again every cycle.
  agent->SetOutputLinkChangeTracking(true);

  if (!config.productionsPath.empty()) {
    if (!agent->LoadProductions(config.productionsPath.c_str(), false)) {
      const char* why = agent->HadError() ? agent->GetLastErrorDescription() : "";
      *error = "Failed to load productions from '" + config.productionsPath + "': " + why;
      return false;
    }
  }

  // Soar's default is to stop a run before the apply phase, which would split
  // each decision across two robot loops: input N would be decided in loop N
  // but applied and output in loop N+1, 20 ms late. Stopping before input
  // makes one run cover input -> propose -> decide -> apply -> output.
  // Set after loading so a productions file cannot undo it.
  agent->ExecuteCommandLine("soar stop-phase input");
  if (!agent->GetLastCommandLineResult()) {
    *error = "Failed to set Soar stop-phase to input";
    return false;
  }
  // The input-link starts empty; WMEs are created the first time their path
  // is sent (Start() re-queues every known path, so a restart sees them all).
  return true;
}

// Destroys the WME at `path` and everything under it, children first.
void SoarHardwareInterface::Impl::DestroySubtree(const std::string& path) {
  std::vector<std::map<std::string, Slot>::iterator> found;
  for (auto it = wmes.lower_bound(path);
       it != wmes.end() && it->first.compare(0, path.size(), path) == 0; ++it) {
    if (IsAtOrUnder(it->first, path)) found.push_back(it);
  }
  // Key order puts a parent before its children, so reverse is leaf-first.
  for (auto r = found.rbegin(); r != found.rend(); ++r) {
    agent->DestroyWME((*r)->second.wme);
    wmes.erase(*r);
  }
}

sml::Identifier* SoarHardwareInterface::Impl::EnsureIdentifier(const std::string& path) {
  auto it = wmes.find(path);
  if (it != wmes.end()) {
    if (it->second.isIdentifier) return it->second.wme->ConvertToIdentifier();
    DestroySubtree(path);  // was a value; becomes a parent
  }
  size_t dot = path.rfind('.');
  sml::Identifier* parent =
      dot == std::string::npos ? agent->GetInputLink() : EnsureIdentifier(path.substr(0, dot));
  const char* attr = path.c_str() + (dot == std::string::npos ? 0 : dot + 1);
  sml::Identifier* id = agent->CreateIdWME(parent, attr);
  wmes[path] = Slot{id, true, ValueType::String};
  return id;
}

void SoarHardwareInterface::Impl::ApplyValue(const std::string& path, const Value& v) {
  auto it = wmes.find(path);
  if (it != wmes.end()) {
    Slot& s = it->second;
    if (!s.isIdentifier && s.type == v.type) {
      switch (v.type) {
        case ValueType::Int:    agent->Update(s.wme->ConvertToIntElement(), v.intValue); break;
        case ValueType::Float:  agent->Update(s.wme->ConvertToFloatElement(), v.floatValue); break;
        case ValueType::String: agent->Update(s.wme->ConvertToStringElement(), v.stringValue.c_str()); break;
      }
      return;
    }
    DestroySubtree(path);  // kind or type changed: recreate
  }
  size_t dot = path.rfind('.');
  sml::Identifier* parent =
      dot == std::string::npos ? agent->GetInputLink() : EnsureIdentifier(path.substr(0, dot));
  const char* attr = path.c_str() + (dot == std::string::npos ? 0 : dot + 1);
  sml::WMElement* wme = nullptr;
  switch (v.type) {
    case ValueType::Int:    wme = agent->CreateIntWME(parent, attr, v.intValue); break;
    case ValueType::Float:  wme = agent->CreateFloatWME(parent, attr, v.floatValue); break;
    case ValueType::String: wme = agent->CreateStringWME(parent, attr, v.stringValue.c_str()); break;
  }
  if (wme != nullptr) wmes[path] = Slot{wme, false, v.type};
}

// Commands are looked up by time tag on the live output-link rather than by
// stored pointer: the agent may already have removed one, which frees it.
bool SoarHardwareInterface::Impl::MarkStatus(long long id, bool success) {
  sml::Identifier* ol = agent->GetOutputLink();
  if (ol == nullptr) return false;
  for (auto it = ol->GetChildrenBegin(); it != ol->GetChildrenEnd(); ++it) {
    sml::WMElement* child = *it;
    if (child->GetTimeTag() != id || !child->IsIdentifier()) continue;
    sml::Identifier* cmd = child->ConvertToIdentifier();
    if (success) cmd->AddStatusComplete(); else cmd->AddStatusError();
    return true;
  }
  return false;
}

int SoarHardwareInterface::Impl::ApplyCompletions(const std::vector<Completion>& done) {
  int applied = 0;
  for (const Completion& c : done) {
    if (MarkStatus(c.id, c.success)) ++applied;
  }
  return applied;
}

// Fills `out` from an output-link command. False (with `why`) if a parameter
// is malformed, a built-in command is missing one, or the name is not accepted.
bool SoarHardwareInterface::Impl::ParseCommand(sml::Identifier* cmd, Command* out, std::string* why) {
  const char* name = cmd->GetAttribute();
  if (name == nullptr) {
    *why = "output-link command without a name";
    return false;
  }
  out->name = name;
  out->id = cmd->GetTimeTag();
  if (!ReadParams(cmd, "", 0, out->name, &out->params, why)) return false;
  bool builtIn = false;
  if (!FillBuiltIn(out, &builtIn, why)) return false;
  if (!builtIn && acceptedCopy.count(out->name) == 0) {
    *why = "unknown command ^" + out->name + " (register it with OnCommand or AcceptCommand)";
    return false;
  }
  return true;
}

// One pass over the top level of the output-link. New identifiers are
// commands; new constants cannot carry ^status, so they are counted, reported
// and otherwise ignored.
void SoarHardwareInterface::Impl::CollectCommands(std::vector<Command>* out, int* rejected,
                                                  int* constants, std::string* warning) {
  sml::Identifier* ol = agent->GetOutputLink();
  if (ol != nullptr) {
    for (auto it = ol->GetChildrenBegin(); it != ol->GetChildrenEnd(); ++it) {
      sml::WMElement* wme = *it;
      if (!wme->IsJustAdded()) continue;
      if (!wme->IsIdentifier()) {
        ++*constants;
        *warning = std::string("ignored constant ^") + wme->GetAttribute() + " " +
                   wme->GetValueAsString() +
                   " on the output-link: a command must be an identifier, e.g. (<ol> ^" +
                   wme->GetAttribute() + " <c>) (<c> ^value " + wme->GetValueAsString() + ")";
        continue;
      }
      sml::Identifier* cmd = wme->ConvertToIdentifier();
      Command parsed;
      std::string why;
      if (ParseCommand(cmd, &parsed, &why)) {
        out->push_back(std::move(parsed));
      } else {
        cmd->AddStatusError();
        ++*rejected;
        *warning = "rejected: " + why;
      }
    }
  }
  agent->ClearOutputLinkChanges();
}

void SoarHardwareInterface::Impl::ShutdownSoar() {
  if (kernel != nullptr) {
    kernel->Shutdown();
    delete kernel;  // also destroys the agent and every client-side WME
  }
  kernel = nullptr;
  agent = nullptr;
  wmes.clear();
  acceptedSeen = -1;
}

void SoarHardwareInterface::Impl::ThreadMain() {
#if defined(__linux__)
  if (config.realtimePriority > 0) {
    sched_param param{};
    param.sched_priority = config.realtimePriority;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);  // best effort
  }
#endif
  std::string error;
  bool ok = InitSoar(&error);
  {
    std::lock_guard<std::mutex> lock(mutex);
    state = ok ? State::Running : State::Failed;
    if (!ok) lastError = error;
  }
  started.notify_all();
  if (!ok) {
    ShutdownSoar();
    return;
  }

  const bool debugger = config.debuggerPort > 0;
  const int decisions = config.decisionsPerInput > 0 ? config.decisionsPerInput : 1;
  const size_t maxPending = config.maxPendingCommands > 0 ? config.maxPendingCommands : 1;
  std::vector<Completion> done;
  std::vector<Command> accepted_;
  std::vector<long long> droppedIds;
  std::vector<std::string> removals;
  std::vector<InputChange> values;   // swapped with pendingValues; elements reused
  size_t valueCount = 0;

  for (;;) {
    bool haveInput = false;
    Clock::time_point inputTime;
    done.clear();
    removals.clear();
    valueCount = 0;
    {
      std::unique_lock<std::mutex> lock(mutex);
      auto ready = [this] { return stopRequested || inputsFresh || !completions.empty(); };
      if (debugger) {
        // Wake periodically so a remote debugger stays responsive.
        wake.wait_for(lock, std::chrono::milliseconds(50), ready);
      } else {
        wake.wait(lock, ready);
      }
      if (stopRequested) break;
      if (inputsFresh) {
        removals.swap(pendingRemovals);
        values.swap(pendingValues);
        valueCount = pendingCount;
        pendingCount = 0;
        inputTime = inputsTime;
        inputsFresh = false;
        haveInput = true;
      }
      if (acceptedSeen != acceptedVersion) {
        acceptedCopy = accepted;  // only when AcceptCommand() changed it
        acceptedSeen = acceptedVersion;
      }
      done.swap(completions);
    }

    if (debugger) kernel->CheckForIncomingCommands();

    int completed = ApplyCompletions(done);
    accepted_.clear();
    droppedIds.clear();
    int rejected = 0;
    int constants = 0;
    std::string warning;
    if (haveInput) {
      for (const std::string& path : removals) DestroySubtree(path);
      for (size_t i = 0; i < valueCount; ++i) ApplyValue(values[i].path, values[i].value);
      agent->Commit();
      agent->RunSelf(decisions);
      CollectCommands(&accepted_, &rejected, &constants, &warning);
    }

    if (haveInput || completed > 0) {
      std::lock_guard<std::mutex> lock(mutex);
      stats.commandsCompleted += completed;
      if (haveInput) {
        for (Command& c : accepted_) commands.push_back(std::move(c));
        if (commands.size() > maxPending) {
          // Oldest first. Each still gets ^status error below, so the agent
          // can clear it instead of waiting on it forever.
          size_t excess = commands.size() - maxPending;
          for (size_t i = 0; i < excess; ++i) droppedIds.push_back(commands[i].id);
          commands.erase(commands.begin(), commands.begin() + excess);
          stats.commandsDropped += static_cast<long long>(excess);
          warning = "dropped " + std::to_string(excess) +
                    " command(s): more than maxPendingCommands waiting; is the robot "
                    "calling PollCommands or DispatchCommands?";
        }
        double micros = std::chrono::duration<double, std::micro>(Clock::now() - inputTime).count();
        stats.inputsProcessed += 1;
        stats.decisionCycles += decisions;
        stats.commandsReceived += static_cast<long long>(accepted_.size());
        stats.commandsRejected += rejected;
        stats.outputConstantsIgnored += constants;
        stats.lastLatencyMicros = micros;
        if (micros > stats.maxLatencyMicros) stats.maxLatencyMicros = micros;
        if (micros > 5000.0) stats.latencyOver5ms += 1;
        stats.meanLatencyMicros += (micros - stats.meanLatencyMicros) / stats.inputsProcessed;
      }
      if (!warning.empty()) lastWarning = warning;
    }
    for (long long id : droppedIds) MarkStatus(id, false);
    // Sends status WMEs added above; they reach the agent on its next cycle.
    if (agent->IsCommitRequired()) agent->Commit();
  }

  ShutdownSoar();
  std::lock_guard<std::mutex> lock(mutex);
  state = State::Stopped;
}

// ---------------------------------------------------------------------------
// Robot side

SoarHardwareInterface::SoarHardwareInterface(const BridgeConfig& config)
    : m_impl(new Impl(config)) {}

SoarHardwareInterface::~SoarHardwareInterface() { Stop(); }

bool SoarHardwareInterface::Start() {
  Impl& s = *m_impl;
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    if (s.state == Impl::State::Running) return true;
  }
  Stop();  // joins a thread left over from a failed start
  {
    std::lock_guard<std::mutex> in(s.inputMutex);
    std::lock_guard<std::mutex> lock(s.mutex);
    s.state = Impl::State::Starting;
    s.stopRequested = false;
    s.inputsFresh = false;
    s.lastError.clear();
    s.lastWarning.clear();
    s.commands.clear();
    s.completions.clear();
    s.stats = BridgeStats();
    // A new kernel has an empty input-link: queue every known value.
    s.pendingRemovals.clear();
    s.QueueAllValues();
  }
  std::unique_lock<std::mutex> lock(s.mutex);
  s.thread = std::thread([&s] { s.ThreadMain(); });
  s.started.wait(lock, [&s] { return s.state != Impl::State::Starting; });
  return s.state == Impl::State::Running;
}

void SoarHardwareInterface::Stop() {
  Impl& s = *m_impl;
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    s.stopRequested = true;
  }
  s.wake.notify_all();
  if (s.thread.joinable()) s.thread.join();
}

bool SoarHardwareInterface::IsRunning() const {
  std::lock_guard<std::mutex> lock(m_impl->mutex);
  return m_impl->state == Impl::State::Running;
}

std::string SoarHardwareInterface::GetLastError() const {
  std::lock_guard<std::mutex> lock(m_impl->mutex);
  return m_impl->lastError;
}

std::string SoarHardwareInterface::GetLastWarning() const {
  std::lock_guard<std::mutex> lock(m_impl->mutex);
  return m_impl->lastWarning;
}

bool SoarHardwareInterface::Impl::Refuse(const std::string& why) {
  std::lock_guard<std::mutex> lock(mutex);
  stats.inputErrors += 1;
  lastWarning = why;
  return false;
}

void SoarHardwareInterface::Impl::MarkDirty(Entries::iterator it) {
  if (it->second.dirty) return;
  it->second.dirty = true;
  dirty.push_back(it);
}

void SoarHardwareInterface::Impl::QueueAllValues() {
  pendingCount = 0;
  for (const auto& e : current) PutChange(&pendingValues, &pendingCount, e.first, e.second.value);
  for (auto it : dirty) it->second.dirty = false;  // all sent anyway
  dirty.clear();
}

bool SoarHardwareInterface::Impl::StageValue(const std::string& path, const Value& value) {
  auto it = current.find(path);
  if (it != current.end()) {
    if (it->second.value == value) return true;  // unchanged: nothing to send
    it->second.value = value;  // reuses string capacity
    MarkDirty(it);
    return true;
  }
  // New path: no ancestor may be a value, and it must not already be a parent.
  for (size_t dot = path.find('.'); dot != std::string::npos; dot = path.find('.', dot + 1)) {
    if (current.count(path.substr(0, dot))) {
      return Refuse("input '" + path + "' refused: '" + path.substr(0, dot) + "' is a value, not a parent");
    }
  }
  auto below = current.lower_bound(path + ".");
  if (below != current.end() && below->first.compare(0, path.size() + 1, path + ".") == 0) {
    return Refuse("input '" + path + "' refused: it already has values under it ('" + below->first + "')");
  }
  MarkDirty(current.emplace(path, Entry{value, false}).first);
  return true;
}

bool SoarHardwareInterface::Impl::SetPath(const std::string& path, Value value) {
  if (!ValidPath(path)) {
    return Refuse("input path '" + path + "' refused: use dot-separated names of letters, "
                  "digits, '-' and '_', each starting with a letter");
  }
  std::lock_guard<std::mutex> lock(inputMutex);
  return StageValue(path, value);
}

bool SoarHardwareInterface::SetNumber(const std::string& path, double value) {
  return m_impl->SetPath(path, Value::FromFloat(value));
}

bool SoarHardwareInterface::SetInt(const std::string& path, long long value) {
  return m_impl->SetPath(path, Value::FromInt(value));
}

bool SoarHardwareInterface::SetString(const std::string& path, const std::string& value) {
  return m_impl->SetPath(path, Value::FromString(value));
}

bool SoarHardwareInterface::SetBool(const std::string& path, bool value) {
  return m_impl->SetPath(path, Value::FromBool(value));
}

bool SoarHardwareInterface::RemoveInput(const std::string& path) {
  Impl& s = *m_impl;
  if (!ValidPath(path)) return s.Refuse("input path '" + path + "' refused: not a valid path");
  std::lock_guard<std::mutex> lock(s.inputMutex);
  // Forget values at or under `path`, including ones not sent yet.
  s.dirty.erase(std::remove_if(s.dirty.begin(), s.dirty.end(),
                               [&](Impl::Entries::iterator e) { return IsAtOrUnder(e->first, path); }),
                s.dirty.end());
  auto it = s.current.lower_bound(path);
  while (it != s.current.end() && it->first.compare(0, path.size(), path) == 0) {
    if (IsAtOrUnder(it->first, path)) it = s.current.erase(it); else ++it;
  }
  // Queued even if nothing is known here: the identifier may still exist on
  // the input-link after its last child was removed.
  s.removed.push_back(path);
  return true;
}

void SoarHardwareInterface::SendInputs() {
  Impl& s = *m_impl;
  Clock::time_point now = Clock::now();
  {
    std::lock_guard<std::mutex> in(s.inputMutex);
    std::lock_guard<std::mutex> lock(s.mutex);
    if (s.inputsFresh) s.stats.inputsOverwritten += 1;
    // Removals first. A removal cancels unapplied values it covers from
    // earlier SendInputs() calls; values staged in this loop that survive in
    // `current` were set after the removal, so they correctly come later.
    for (std::string& path : s.removed) {
      size_t kept = 0;
      for (size_t i = 0; i < s.pendingCount; ++i) {
        if (IsAtOrUnder(s.pendingValues[i].path, path)) continue;
        if (kept != i) std::swap(s.pendingValues[kept], s.pendingValues[i]);
        ++kept;
      }
      s.pendingCount = kept;
      s.pendingRemovals.push_back(std::move(path));
    }
    s.removed.clear();
    if (s.pendingCount + s.dirty.size() > s.current.size() + 16) {
      // Soar has fallen several loops behind: send one full snapshot instead
      // of a growing list of changes. Unchanged values do not blink.
      s.QueueAllValues();
    } else {
      for (auto it : s.dirty) {
        PutChange(&s.pendingValues, &s.pendingCount, it->first, it->second.value);
        it->second.dirty = false;
      }
      s.dirty.clear();
    }
    s.inputsTime = now;
    s.inputsFresh = true;
    s.stats.inputsSubmitted += 1;
  }
  s.wake.notify_one();
}

void SoarHardwareInterface::SetInputs(const RobotInputs& in) {
  static const std::string kMatchTime = "match.time", kMatchMode = "match.mode",
      kMatchAlliance = "match.alliance", kPoseX = "drive.pose.x", kPoseY = "drive.pose.y",
      kHeading = "drive.pose.heading", kVelocityX = "drive.velocity.vx",
      kVelocityY = "drive.velocity.vy", kTargetDetected = "vision.target-detected",
      kTargetDistance = "vision.target-distance", kTargetYaw = "vision.target-yaw",
      kHasPiece = "intake.has-piece", kElevatorHeight = "elevator.height";
  Impl& s = *m_impl;
  {
    std::lock_guard<std::mutex> lock(s.inputMutex);
    s.StageValue(kMatchTime, Value::FromFloat(in.matchTime));
    s.StageValue(kMatchMode, Value::FromString(ModeName(in.mode)));
    s.StageValue(kMatchAlliance, Value::FromString(AllianceName(in.alliance)));
    s.StageValue(kPoseX, Value::FromFloat(in.poseX));
    s.StageValue(kPoseY, Value::FromFloat(in.poseY));
    s.StageValue(kHeading, Value::FromFloat(in.heading));
    s.StageValue(kVelocityX, Value::FromFloat(in.velocityX));
    s.StageValue(kVelocityY, Value::FromFloat(in.velocityY));
    s.StageValue(kTargetDetected, Value::FromBool(in.targetDetected));
    s.StageValue(kTargetDistance, Value::FromFloat(in.targetDistance));
    s.StageValue(kTargetYaw, Value::FromFloat(in.targetYaw));
    s.StageValue(kHasPiece, Value::FromBool(in.hasPiece));
    s.StageValue(kElevatorHeight, Value::FromFloat(in.elevatorHeight));
  }
  SendInputs();
}

bool SoarHardwareInterface::AcceptCommand(const std::string& name) {
  Impl& s = *m_impl;
  std::lock_guard<std::mutex> lock(s.mutex);
  if (!ValidPath(name, true)) {
    s.lastWarning = "command name '" + name + "' refused: use letters, digits, '-' and '_', "
                    "starting with a letter";
    return false;
  }
  if (s.accepted.insert(name).second) s.acceptedVersion += 1;
  return true;
}

bool SoarHardwareInterface::OnCommand(const std::string& name, CommandCallback handler) {
  if (!AcceptCommand(name)) return false;
  m_impl->handlers[name] = std::move(handler);
  return true;
}

std::vector<Command> SoarHardwareInterface::PollCommands() {
  std::vector<Command> out;
  std::lock_guard<std::mutex> lock(m_impl->mutex);
  out.swap(m_impl->commands);
  return out;
}

void SoarHardwareInterface::CompleteCommand(long long commandId, bool success) {
  Impl& s = *m_impl;
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    if (s.state != Impl::State::Running) return;
    s.completions.push_back({commandId, success});
  }
  s.wake.notify_one();
}

void SoarHardwareInterface::RejectCommand(long long commandId, const std::string& reason) {
  Impl& s = *m_impl;
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    s.stats.commandsUnhandled += 1;
    s.lastWarning = reason;
    if (s.state != Impl::State::Running) return;
    s.completions.push_back({commandId, false});
  }
  s.wake.notify_one();
}

int SoarHardwareInterface::Impl::Dispatch(CommandHandler* handler) {
  std::vector<Command> polled;
  {
    std::lock_guard<std::mutex> lock(mutex);
    polled.swap(commands);
  }
  std::exception_ptr failure;
  for (const Command& c : polled) {
    bool ok = true;
    bool handled = true;
    try {
      auto it = handlers.find(c.name);
      if (it != handlers.end()) {
        it->second(c);
      } else if (handler != nullptr && c.type != CommandType::Custom) {
        switch (c.type) {
          case CommandType::Move:        handler->Move(c.vx, c.vy, c.omega); break;
          case CommandType::TurnTo:      handler->TurnTo(c.heading); break;
          case CommandType::SetElevator: handler->SetElevator(c.height); break;
          case CommandType::RunIntake:   handler->RunIntake(c.speed); break;
          case CommandType::Shoot:       handler->Shoot(c.execute); break;
          case CommandType::Custom:      break;
        }
      } else {
        handled = false;
      }
    } catch (...) {
      // Every polled command still gets a status so none is stranded on the
      // output-link; the first failure is rethrown once all are handled.
      ok = false;
      if (!failure) failure = std::current_exception();
    }
    // CompleteCommand() / RejectCommand(), inlined to reach this Impl.
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!handled) {
        ok = false;
        stats.commandsUnhandled += 1;
        lastWarning = "no handler for ^" + c.name + " (register one with OnCommand)";
      }
      if (state == State::Running) completions.push_back({c.id, ok});
    }
    wake.notify_one();
  }
  if (failure) std::rethrow_exception(failure);
  return static_cast<int>(polled.size());
}

int SoarHardwareInterface::DispatchCommands(CommandHandler& handler) {
  return m_impl->Dispatch(&handler);
}

int SoarHardwareInterface::DispatchCommands() { return m_impl->Dispatch(nullptr); }

BridgeStats SoarHardwareInterface::GetStats() const {
  std::lock_guard<std::mutex> lock(m_impl->mutex);
  return m_impl->stats;
}

}  // namespace soarbridge
