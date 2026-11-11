#include "soarbridge/SoarHardwareInterface.h"

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <mutex>
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

// Soar has no boolean type; booleans are the string constants true/false.
const char* BoolName(bool value) { return value ? "true" : "false"; }

bool ReadFloat(sml::Identifier* cmd, const char* attribute, double* out) {
  const char* text = cmd->GetParameterValue(attribute);
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  double value = std::strtod(text, &end);
  if (end == text || *end != '\0') return false;
  *out = value;
  return true;
}

bool ReadBool(sml::Identifier* cmd, const char* attribute, bool* out) {
  const char* text = cmd->GetParameterValue(attribute);
  if (text == nullptr) return false;
  if (!std::strcmp(text, "true") || !std::strcmp(text, "1")) { *out = true; return true; }
  if (!std::strcmp(text, "false") || !std::strcmp(text, "0")) { *out = false; return true; }
  return false;
}

// Fills `out` from an output-link command. False if the name is unknown or a
// required parameter is missing or malformed.
bool ParseCommand(sml::Identifier* cmd, Command* out) {
  const char* name = cmd->GetCommandName();
  if (name == nullptr) return false;
  out->id = cmd->GetTimeTag();
  if (!std::strcmp(name, "move")) {
    out->type = CommandType::Move;
    return ReadFloat(cmd, "vx", &out->vx) && ReadFloat(cmd, "vy", &out->vy) &&
           ReadFloat(cmd, "omega", &out->omega);
  }
  if (!std::strcmp(name, "turn-to")) {
    out->type = CommandType::TurnTo;
    return ReadFloat(cmd, "heading", &out->heading);
  }
  if (!std::strcmp(name, "set-elevator")) {
    out->type = CommandType::SetElevator;
    return ReadFloat(cmd, "height", &out->height);
  }
  if (!std::strcmp(name, "run-intake")) {
    out->type = CommandType::RunIntake;
    return ReadFloat(cmd, "speed", &out->speed);
  }
  if (!std::strcmp(name, "shoot")) {
    out->type = CommandType::Shoot;
    return ReadBool(cmd, "execute", &out->execute);
  }
  return false;
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

  // Everything below is guarded by `mutex`. Neither thread holds it for more
  // than a struct copy or a vector swap.
  mutable std::mutex mutex;
  std::condition_variable wake;      // robot -> Soar: new input, completion, stop
  std::condition_variable started;   // Soar -> Start(): init finished
  enum class State { Stopped, Starting, Running, Failed } state = State::Stopped;
  bool stopRequested = false;
  std::string lastError;

  RobotInputs inputs;
  bool inputsFresh = false;
  Clock::time_point inputsTime;

  std::vector<Command> commands;         // Soar -> robot
  std::vector<Completion> completions;   // robot -> Soar
  BridgeStats stats;

  // --- Soar thread only ---
  sml::Kernel* kernel = nullptr;
  sml::Agent* agent = nullptr;
  struct {
    sml::FloatElement* matchTime;
    sml::StringElement* matchMode;
    sml::StringElement* matchAlliance;
    sml::FloatElement* poseX;
    sml::FloatElement* poseY;
    sml::FloatElement* heading;
    sml::FloatElement* velocityX;
    sml::FloatElement* velocityY;
    sml::StringElement* targetDetected;
    sml::FloatElement* targetDistance;
    sml::FloatElement* targetYaw;
    sml::StringElement* hasPiece;
    sml::FloatElement* elevatorHeight;
  } wme{};

  void ThreadMain();
  bool InitSoar(std::string* error);
  void WriteInputs(const RobotInputs& in);
  int ApplyCompletions(const std::vector<Completion>& done);
  void CollectCommands(std::vector<Command>* accepted, int* rejected);
  void ShutdownSoar();
};

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

  sml::Identifier* il = agent->GetInputLink();
  sml::Identifier* match = agent->CreateIdWME(il, "match");
  sml::Identifier* drive = agent->CreateIdWME(il, "drive");
  sml::Identifier* pose = agent->CreateIdWME(drive, "pose");
  sml::Identifier* velocity = agent->CreateIdWME(drive, "velocity");
  sml::Identifier* vision = agent->CreateIdWME(il, "vision");
  sml::Identifier* intake = agent->CreateIdWME(il, "intake");
  sml::Identifier* elevator = agent->CreateIdWME(il, "elevator");

  const RobotInputs init;
  wme.matchTime = agent->CreateFloatWME(match, "time", init.matchTime);
  wme.matchMode = agent->CreateStringWME(match, "mode", ModeName(init.mode));
  wme.matchAlliance = agent->CreateStringWME(match, "alliance", AllianceName(init.alliance));
  wme.poseX = agent->CreateFloatWME(pose, "x", init.poseX);
  wme.poseY = agent->CreateFloatWME(pose, "y", init.poseY);
  wme.heading = agent->CreateFloatWME(pose, "heading", init.heading);
  wme.velocityX = agent->CreateFloatWME(velocity, "vx", init.velocityX);
  wme.velocityY = agent->CreateFloatWME(velocity, "vy", init.velocityY);
  wme.targetDetected = agent->CreateStringWME(vision, "target-detected", BoolName(init.targetDetected));
  wme.targetDistance = agent->CreateFloatWME(vision, "target-distance", init.targetDistance);
  wme.targetYaw = agent->CreateFloatWME(vision, "target-yaw", init.targetYaw);
  wme.hasPiece = agent->CreateStringWME(intake, "has-piece", BoolName(init.hasPiece));
  wme.elevatorHeight = agent->CreateFloatWME(elevator, "height", init.elevatorHeight);
  agent->Commit();
  return true;
}

void SoarHardwareInterface::Impl::WriteInputs(const RobotInputs& in) {
  agent->Update(wme.matchTime, in.matchTime);
  agent->Update(wme.matchMode, ModeName(in.mode));
  agent->Update(wme.matchAlliance, AllianceName(in.alliance));
  agent->Update(wme.poseX, in.poseX);
  agent->Update(wme.poseY, in.poseY);
  agent->Update(wme.heading, in.heading);
  agent->Update(wme.velocityX, in.velocityX);
  agent->Update(wme.velocityY, in.velocityY);
  agent->Update(wme.targetDetected, BoolName(in.targetDetected));
  agent->Update(wme.targetDistance, in.targetDistance);
  agent->Update(wme.targetYaw, in.targetYaw);
  agent->Update(wme.hasPiece, BoolName(in.hasPiece));
  agent->Update(wme.elevatorHeight, in.elevatorHeight);
}

// Commands are looked up by time tag on the live output-link rather than by
// stored pointer: the agent may already have removed one, which frees it.
int SoarHardwareInterface::Impl::ApplyCompletions(const std::vector<Completion>& done) {
  sml::Identifier* ol = agent->GetOutputLink();
  if (ol == nullptr) return 0;
  int applied = 0;
  for (const Completion& c : done) {
    int n = ol->GetNumberChildren();
    for (int i = 0; i < n; ++i) {
      sml::WMElement* child = ol->GetChild(i);
      if (child == nullptr || child->GetTimeTag() != c.id || !child->IsIdentifier()) continue;
      sml::Identifier* cmd = child->ConvertToIdentifier();
      if (c.success) cmd->AddStatusComplete(); else cmd->AddStatusError();
      ++applied;
      break;
    }
  }
  return applied;
}

void SoarHardwareInterface::Impl::CollectCommands(std::vector<Command>* accepted, int* rejected) {
  int n = agent->GetNumberCommands();
  for (int i = 0; i < n; ++i) {
    sml::Identifier* cmd = agent->GetCommand(i);
    if (cmd == nullptr) continue;
    Command parsed;
    if (ParseCommand(cmd, &parsed)) {
      accepted->push_back(parsed);
    } else {
      cmd->AddStatusError();
      ++*rejected;
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
  wme = {};
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
  std::vector<Command> accepted;

  for (;;) {
    RobotInputs in;
    bool haveInput = false;
    Clock::time_point inputTime;
    done.clear();
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
        in = inputs;
        inputTime = inputsTime;
        inputsFresh = false;
        haveInput = true;
      }
      done.swap(completions);
    }

    if (debugger) kernel->CheckForIncomingCommands();

    int completed = ApplyCompletions(done);
    accepted.clear();
    int rejected = 0;
    if (haveInput) {
      WriteInputs(in);
      agent->Commit();
      agent->RunSelf(decisions);
      CollectCommands(&accepted, &rejected);
    }
    // Sends status WMEs added above; they reach the agent on its next cycle.
    if (agent->IsCommitRequired()) agent->Commit();

    if (haveInput || completed > 0) {
      std::lock_guard<std::mutex> lock(mutex);
      stats.commandsCompleted += completed;
      if (haveInput) {
        for (const Command& c : accepted) commands.push_back(c);
        if (commands.size() > maxPending) {
          size_t excess = commands.size() - maxPending;
          commands.erase(commands.begin(), commands.begin() + excess);
          stats.commandsDropped += static_cast<long long>(excess);
        }
        double micros = std::chrono::duration<double, std::micro>(Clock::now() - inputTime).count();
        stats.inputsProcessed += 1;
        stats.decisionCycles += decisions;
        stats.commandsReceived += static_cast<long long>(accepted.size());
        stats.commandsRejected += rejected;
        stats.lastLatencyMicros = micros;
        if (micros > stats.maxLatencyMicros) stats.maxLatencyMicros = micros;
        if (micros > 5000.0) stats.latencyOver5ms += 1;
        stats.meanLatencyMicros += (micros - stats.meanLatencyMicros) / stats.inputsProcessed;
      }
    }
  }

  ShutdownSoar();
  std::lock_guard<std::mutex> lock(mutex);
  state = State::Stopped;
}

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
  std::unique_lock<std::mutex> lock(s.mutex);
  s.state = Impl::State::Starting;
  s.stopRequested = false;
  s.inputsFresh = false;
  s.lastError.clear();
  s.commands.clear();
  s.completions.clear();
  s.stats = BridgeStats();
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

void SoarHardwareInterface::SetInputs(const RobotInputs& inputs) {
  Impl& s = *m_impl;
  Clock::time_point now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    if (s.inputsFresh) s.stats.inputsOverwritten += 1;
    s.inputs = inputs;
    s.inputsTime = now;
    s.inputsFresh = true;
    s.stats.inputsSubmitted += 1;
  }
  s.wake.notify_one();
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

int SoarHardwareInterface::DispatchCommands(CommandHandler& handler) {
  std::vector<Command> polled = PollCommands();
  std::exception_ptr failure;
  for (const Command& c : polled) {
    bool ok = true;
    try {
      switch (c.type) {
        case CommandType::Move:        handler.Move(c.vx, c.vy, c.omega); break;
        case CommandType::TurnTo:      handler.TurnTo(c.heading); break;
        case CommandType::SetElevator: handler.SetElevator(c.height); break;
        case CommandType::RunIntake:   handler.RunIntake(c.speed); break;
        case CommandType::Shoot:       handler.Shoot(c.execute); break;
      }
    } catch (...) {
      // Every polled command still gets a status so none is stranded on the
      // output-link; the first failure is rethrown once all are handled.
      ok = false;
      if (!failure) failure = std::current_exception();
    }
    CompleteCommand(c.id, ok);
  }
  if (failure) std::rethrow_exception(failure);
  return static_cast<int>(polled.size());
}

BridgeStats SoarHardwareInterface::GetStats() const {
  std::lock_guard<std::mutex> lock(m_impl->mutex);
  return m_impl->stats;
}

}  // namespace soarbridge
