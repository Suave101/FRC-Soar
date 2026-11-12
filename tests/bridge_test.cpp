// End-to-end test of SoarHardwareInterface against a real Soar kernel.
//
//   bridge_test <agents dir> <tests dir> [--realtime]
//
// The match simulation runs 7500 loops (2.5 minutes of 20 ms ticks). By
// default the tick period is shortened to 1 ms; --realtime uses the true 20 ms
// period and takes the full 150 s.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "soarbridge/SoarHardwareInterface.h"

using namespace soarbridge;
using Clock = std::chrono::steady_clock;

static int g_failures = 0;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
      ++g_failures;                                                        \
    }                                                                      \
  } while (0)

static double Micros(Clock::duration d) {
  return std::chrono::duration<double, std::micro>(d).count();
}

// Resident set size in kB (Linux); 0 where unavailable or meaningless.
static long RssKb() {
#if defined(__SANITIZE_ADDRESS__)
  return 0;  // ASan's quarantine dominates RSS; LeakSanitizer does the checking
#endif
  std::ifstream statm("/proc/self/statm");
  long pages = 0, resident = 0;
  if (!(statm >> pages >> resident)) return 0;
  return resident * 4;
}

// Submits `in` and returns the commands produced for it, waiting at most 1 s.
static std::vector<Command> Tick(SoarHardwareInterface& soar, const RobotInputs& in) {
  long long target = soar.GetStats().inputsProcessed + 1;
  soar.SetInputs(in);
  for (int i = 0; i < 1000 && soar.GetStats().inputsProcessed < target; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return soar.PollCommands();
}

static const Command* Find(const std::vector<Command>& cmds, CommandType type) {
  for (const Command& c : cmds) if (c.type == type) return &c;
  return nullptr;
}

struct Recorder : CommandHandler {
  int moves = 0, turns = 0, elevators = 0, intakes = 0, shots = 0;
  double lastOmega = 0;
  void Move(double, double, double omega) override { ++moves; lastOmega = omega; }
  void TurnTo(double) override { ++turns; }
  void SetElevator(double) override { ++elevators; }
  void RunIntake(double) override { ++intakes; }
  void Shoot(bool) override { ++shots; }
};

static void TestStartFailure() {
  std::printf("start failure\n");
  BridgeConfig cfg;
  cfg.productionsPath = "/nonexistent/agent.soar";
  SoarHardwareInterface soar(cfg);
  CHECK(!soar.Start());
  CHECK(!soar.IsRunning());
  CHECK(!soar.GetLastError().empty());
  soar.SetInputs(RobotInputs());  // must be harmless when not running
  CHECK(soar.PollCommands().empty());
}

static void TestMapping(const std::string& agents) {
  std::printf("io mapping\n");
  BridgeConfig cfg;
  cfg.productionsPath = agents + "/default.soar";
  SoarHardwareInterface soar(cfg);
  CHECK(soar.Start());
  CHECK(soar.IsRunning());

  RobotInputs in;  // disabled: agent must stay quiet
  for (int i = 0; i < 5; ++i) CHECK(Tick(soar, in).empty());

  in.mode = MatchMode::Teleop;
  in.targetDetected = true;
  in.targetYaw = 10.0;
  in.targetDistance = 2.0;
  std::vector<Command> cmds;
  int ticks = 0;
  for (; ticks < 5 && cmds.empty(); ++ticks) cmds = Tick(soar, in);
  std::printf("  ticks from input change to command: %d\n", ticks);
  CHECK(ticks == 1);  // the agent reacts within the decision cycle that sees the input
  const Command* move = Find(cmds, CommandType::Move);
  const Command* intake = Find(cmds, CommandType::RunIntake);
  CHECK(move != nullptr);
  CHECK(intake != nullptr);
  if (move) {
    CHECK(std::fabs(move->vx - 1.0) < 1e-9);
    CHECK(std::fabs(move->vy) < 1e-9);
    CHECK(std::fabs(move->omega + 0.3) < 1e-9);
  }
  if (intake) CHECK(std::fabs(intake->speed - 1.0) < 1e-9);
  if (move) {
    // The typed fields sit on top of the generic name/params.
    CHECK(move->name == "move");
    CHECK(move->params.size() == 3);
    CHECK(std::fabs(move->GetNumber("vx") - 1.0) < 1e-9);
  }
  // The typed layer owns ^match.* as a parent, so a value there is refused.
  CHECK(!soar.SetNumber("match", 1.0));
  CHECK(soar.GetStats().inputErrors == 1);

  // Until completed, the command stays on the output-link and is not reissued.
  for (int i = 0; i < 5; ++i) CHECK(Tick(soar, in).empty());

  // Completing it lets the agent clear it and issue a fresh one.
  for (const Command& c : cmds) soar.CompleteCommand(c.id);
  in.targetYaw = -20.0;
  cmds.clear();
  for (int i = 0; i < 10 && Find(cmds, CommandType::Move) == nullptr; ++i) cmds = Tick(soar, in);
  move = Find(cmds, CommandType::Move);
  CHECK(move != nullptr);
  if (move) CHECK(std::fabs(move->omega - 0.6) < 1e-9);
  CHECK(soar.GetStats().commandsCompleted == 2);

  // Holding a piece -> shoot, delivered through the handler interface.
  in.hasPiece = true;
  Recorder rec;
  for (int i = 0; i < 10 && rec.shots == 0; ++i) {
    soar.SetInputs(in);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    soar.DispatchCommands(rec);
  }
  CHECK(rec.shots >= 1);

  soar.Stop();
  CHECK(!soar.IsRunning());
  soar.Stop();  // idempotent
}

static void TestBadCommands(const std::string& tests) {
  std::printf("rejected commands\n");
  BridgeConfig cfg;
  cfg.productionsPath = tests + "/bad_commands.soar";
  SoarHardwareInterface soar(cfg);
  CHECK(soar.Start());
  std::vector<Command> all;
  for (int i = 0; i < 5; ++i) {
    std::vector<Command> c = Tick(soar, RobotInputs());
    all.insert(all.end(), c.begin(), c.end());
  }
  BridgeStats s0 = soar.GetStats();
  std::printf("  %zu commands, %lld received, %lld rejected; %s\n", all.size(), s0.commandsReceived,
              s0.commandsRejected, soar.GetLastWarning().c_str());
  CHECK(all.size() == 1);
  if (!all.empty()) {
    CHECK(all[0].type == CommandType::SetElevator);
    CHECK(std::fabs(all[0].height - 0.5) < 1e-9);
  }
  BridgeStats s = soar.GetStats();
  CHECK(s.commandsReceived == 1);
  CHECK(s.commandsRejected == 2);
}

// --- Generic (path-based) I/O, against tests/generic_io.soar ---------------

// Sends the staged inputs and returns the commands produced, waiting at most 1 s.
static std::vector<Command> Send(SoarHardwareInterface& soar) {
  long long target = soar.GetStats().inputsProcessed + 1;
  soar.SendInputs();
  for (int i = 0; i < 1000 && soar.GetStats().inputsProcessed < target; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return soar.PollCommands();
}

static const Command* Named(const std::vector<Command>& cmds, const std::string& name, long long seq) {
  for (const Command& c : cmds) {
    Value s = c.GetParam("seq");
    if (c.name == name && s.type == ValueType::Int && s.intValue == seq) return &c;
  }
  return nullptr;
}

// Sets ^test.seq and ticks until the agent's ^report for it arrives,
// completing every command on the way. Returns all of them.
static std::vector<Command> Burst(SoarHardwareInterface& soar, long long seq) {
  CHECK(soar.SetInt("test.seq", seq));
  std::vector<Command> all;
  for (int i = 0; i < 10 && Named(all, "report", seq) == nullptr; ++i) {
    for (const Command& c : Send(soar)) {
      soar.CompleteCommand(c.id);
      all.push_back(c);
    }
  }
  CHECK(Named(all, "report", seq) != nullptr);
  return all;
}

static void TestGenericIO(const std::string& tests) {
  std::printf("generic io\n");
  BridgeConfig cfg;
  cfg.productionsPath = tests + "/generic_io.soar";
  SoarHardwareInterface soar(cfg);

  // Path syntax is checked on the robot thread, even before Start().
  for (const char* bad : {"", "a..b", "^a", "a b", "1a", "a.", ".a", "a.b|c"}) {
    CHECK(!soar.SetNumber(bad, 1.0));
  }
  CHECK(soar.GetStats().inputErrors == 8);
  CHECK(!soar.GetLastWarning().empty());

  CHECK(soar.Start());
  for (const char* name : {"report", "beam", "angle", "has-piece", "no-vision"}) {
    CHECK(soar.AcceptCommand(name));
  }
  CHECK(!soar.AcceptCommand("ground-intake.state"));  // one segment only

  CHECK(soar.SetString("match.mode", "teleop"));
  CHECK(soar.SetInt("ground-intake.beam-break", 1));
  CHECK(soar.SetBool("ground-intake.has-piece", true));
  CHECK(soar.SetNumber("arm.angle", 30.5));
  CHECK(!soar.SetNumber("arm.angle.x", 1.0));    // arm.angle is a value
  CHECK(!soar.SetNumber("ground-intake", 1.0));  // has values under it
  CHECK(soar.GetStats().inputErrors == 2);

  std::vector<Command> b = Burst(soar, 1);
  const Command* report = Named(b, "report", 1);
  if (report) {
    CHECK(report->type == CommandType::Custom);
    CHECK(report->params.size() == 4);  // seq, mode, target.x, target.y
    CHECK(report->GetString("mode") == "teleop");
    CHECK(report->GetParam("target.x").type == ValueType::Float);
    CHECK(std::fabs(report->GetNumber("target.x") - 1.5) < 1e-9);
    CHECK(report->GetParam("target.y").type == ValueType::Int);
    CHECK(report->GetParam("target.y").intValue == -2);
    CHECK(!report->Has("status"));
  }
  const Command* beam = Named(b, "beam", 1);
  CHECK(beam != nullptr);
  if (beam) {
    CHECK(beam->GetParam("value").type == ValueType::Int);  // SetInt -> int WME
    CHECK(beam->GetParam("value").intValue == 1);
  }
  const Command* angle = Named(b, "angle", 1);
  CHECK(angle != nullptr);
  if (angle) {
    CHECK(angle->GetParam("value").type == ValueType::Float);
    CHECK(std::fabs(angle->GetNumber("value") - 30.5) < 1e-9);
  }
  CHECK(Named(b, "has-piece", 1) != nullptr);  // SetBool -> "true"
  CHECK(Named(b, "no-vision", 1) != nullptr);
  CHECK(Named(b, "mystery", 1) == nullptr);
  CHECK(Named(b, "dup", 1) == nullptr);
  BridgeStats s = soar.GetStats();
  CHECK(s.commandsRejected == 2);         // ^mystery (unregistered), ^dup (two ^v)
  CHECK(s.outputConstantsIgnored == 1);   // ^legacy closed
  std::printf("  last warning: %s\n", soar.GetLastWarning().c_str());

  // A subtree appears, another is removed, values change.
  CHECK(soar.SetNumber("vision.distance", 2.0));
  CHECK(soar.RemoveInput("arm"));
  CHECK(soar.SetInt("ground-intake.beam-break", 0));
  CHECK(soar.SetBool("ground-intake.has-piece", false));
  b = Burst(soar, 2);
  CHECK(Named(b, "no-vision", 2) == nullptr);
  CHECK(Named(b, "angle", 2) == nullptr);
  CHECK(Named(b, "has-piece", 2) == nullptr);
  beam = Named(b, "beam", 2);
  CHECK(beam != nullptr);
  if (beam) CHECK(beam->GetParam("value").intValue == 0);

  // Removed parent; a removed path comes back with a different type.
  CHECK(soar.RemoveInput("vision"));
  CHECK(soar.SetString("arm.angle", "up"));
  b = Burst(soar, 3);
  CHECK(Named(b, "no-vision", 3) != nullptr);
  angle = Named(b, "angle", 3);
  CHECK(angle != nullptr);
  if (angle) CHECK(angle->GetParam("value").type == ValueType::String && angle->GetString("value") == "up");

  // Type change on a live path (string -> float).
  CHECK(soar.SetNumber("arm.angle", -5.0));
  b = Burst(soar, 4);
  angle = Named(b, "angle", 4);
  CHECK(angle != nullptr);
  if (angle) CHECK(angle->GetParam("value").type == ValueType::Float && angle->GetNumber("value") == -5.0);

  // A parent becomes a value within one snapshot.
  CHECK(soar.RemoveInput("arm"));
  CHECK(soar.SetNumber("arm", 3.0));
  b = Burst(soar, 5);
  CHECK(Named(b, "angle", 5) == nullptr);

  // A restart re-sends every known value to the new kernel.
  soar.Stop();
  CHECK(soar.Start());
  b = Burst(soar, 6);
  report = Named(b, "report", 6);
  if (report) CHECK(report->GetString("mode") == "teleop");
  CHECK(Named(b, "beam", 6) != nullptr);  // AcceptCommand survives a restart

  // Handlers by name; an accepted command with no handler gets ^status error.
  int reports = 0;
  std::string mode;
  CHECK(soar.OnCommand("report", [&](const Command& c) {
    ++reports;
    mode = c.GetString("mode");
  }));
  CHECK(soar.SetInt("test.seq", 7));
  for (int i = 0; i < 20 && reports == 0; ++i) {
    soar.SendInputs();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    soar.DispatchCommands();
  }
  CHECK(reports == 1);
  CHECK(mode == "teleop");
  s = soar.GetStats();
  CHECK(s.commandsUnhandled >= 1);  // ^beam: accepted but no handler
  CHECK(s.commandsRejected == 0);   // seq 7 issues nothing the bridge refuses
  std::printf("  unhandled: %lld, warning: %s\n", s.commandsUnhandled, soar.GetLastWarning().c_str());
}

// Commands dropped past maxPendingCommands get ^status error, so the agent
// can clear and re-issue them instead of waiting on them forever.
static void TestDropped(const std::string& tests) {
  std::printf("dropped commands\n");
  BridgeConfig cfg;
  cfg.productionsPath = tests + "/generic_io.soar";
  cfg.maxPendingCommands = 1;
  SoarHardwareInterface soar(cfg);
  CHECK(soar.Start());
  CHECK(soar.AcceptCommand("ping-a"));
  CHECK(soar.AcceptCommand("ping-b"));
  CHECK(soar.SetBool("test.ping", true));
  for (int i = 0; i < 12; ++i) {
    long long target = soar.GetStats().inputsProcessed + 1;
    soar.SendInputs();  // never polled
    for (int w = 0; w < 1000 && soar.GetStats().inputsProcessed < target; ++w) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  BridgeStats s = soar.GetStats();
  std::printf("  12 loops without polling: %lld received, %lld dropped\n",
              s.commandsReceived, s.commandsDropped);
  // Without the error status the agent would be stuck at 2 received, 1 dropped.
  CHECK(s.commandsReceived >= 4);
  CHECK(s.commandsDropped >= 3);
  CHECK(soar.PollCommands().size() <= 1);
  CHECK(soar.GetLastWarning().find("dropped") != std::string::npos);
}

static void TestRestart(const std::string& agents) {
  std::printf("start/stop cycles\n");
  BridgeConfig cfg;
  cfg.productionsPath = agents + "/default.soar";
  RobotInputs in;
  in.mode = MatchMode::Autonomous;
  in.targetDetected = true;
  {
    SoarHardwareInterface soar(cfg);
    for (int i = 0; i < 5; ++i) {
      CHECK(soar.Start());
      std::vector<Command> cmds;
      for (int t = 0; t < 5 && cmds.empty(); ++t) cmds = Tick(soar, in);
      CHECK(!cmds.empty());
      soar.Stop();
    }
  }
  long before = RssKb();
  for (int i = 0; i < 20; ++i) {
    SoarHardwareInterface soar(cfg);
    CHECK(soar.Start());
    Tick(soar, in);
    // destructor stops with a command still pending
  }
  long after = RssKb();
  std::printf("  RSS across 20 kernel lifecycles: %ld kB -> %ld kB\n", before, after);
  CHECK(after - before < 2048);
}

// A full match: 15 s autonomous, 135 s teleop, 50 Hz. Mimics a robot loop:
// set inputs, do other work, then dispatch commands in the same loop.
static void TestMatch(const std::string& agents, bool realtime) {
  const int kTicks = 7500;
  const auto period = realtime ? std::chrono::milliseconds(20) : std::chrono::milliseconds(1);
  std::printf("match simulation (%d ticks, %s period)\n", kTicks, realtime ? "20 ms" : "1 ms");

  BridgeConfig cfg;
  cfg.productionsPath = agents + "/default.soar";
  SoarHardwareInterface soar(cfg);
  CHECK(soar.Start());

  Recorder rec;
  RobotInputs in;
  in.alliance = Alliance::Red;
  double worstCallMicros = 0, worstSetMicros = 0, worstDispatchMicros = 0;
  double totalSetMicros = 0, totalDispatchMicros = 0;
  long rssWarm = 0;
  auto next = Clock::now();

  for (int tick = 0; tick < kTicks; ++tick) {
    double t = tick * 0.02;
    in.mode = t < 15.0 ? MatchMode::Autonomous : MatchMode::Teleop;
    in.matchTime = 150.0 - t;
    in.poseX = 8.0 + 4.0 * std::sin(t * 0.3);
    in.poseY = 4.0 + 2.0 * std::cos(t * 0.2);
    in.heading = std::fmod(t * 30.0, 360.0) - 180.0;
    in.velocityX = 1.2 * std::cos(t * 0.3);
    in.velocityY = -0.4 * std::sin(t * 0.2);
    in.targetDetected = (tick / 40) % 3 != 0;
    in.targetDistance = 3.0 + std::sin(t);
    in.targetYaw = 15.0 * std::sin(t * 1.7);
    in.hasPiece = (tick / 150) % 2 == 1;
    in.elevatorHeight = 0.6 + 0.5 * std::sin(t * 0.5);

    auto a = Clock::now();
    soar.SetInputs(in);
    auto b = Clock::now();
    // The rest of the robot loop runs here; Soar works in parallel.
    std::this_thread::sleep_for(period / 2);
    auto c = Clock::now();
    soar.DispatchCommands(rec);
    auto d = Clock::now();
    worstCallMicros = std::max({worstCallMicros, Micros(b - a), Micros(d - c)});
    worstSetMicros = std::max(worstSetMicros, Micros(b - a));
    worstDispatchMicros = std::max(worstDispatchMicros, Micros(d - c));
    totalSetMicros += Micros(b - a);
    totalDispatchMicros += Micros(d - c);

    if (tick == 1500) rssWarm = RssKb();  // after 30 s, caches are populated
    next += period;
    std::this_thread::sleep_until(next);
  }
  long rssEnd = RssKb();
  // Pick up anything issued for the final input before comparing counts.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  soar.DispatchCommands(rec);
  BridgeStats s = soar.GetStats();
  soar.Stop();

  std::printf("  inputs: %lld submitted, %lld processed, %lld overwritten\n",
              s.inputsSubmitted, s.inputsProcessed, s.inputsOverwritten);
  std::printf("  commands: %lld received, %lld completed, %lld rejected, %lld dropped\n",
              s.commandsReceived, s.commandsCompleted, s.commandsRejected, s.commandsDropped);
  std::printf("  handler calls: move=%d intake=%d shoot=%d\n", rec.moves, rec.intakes, rec.shots);
  std::printf("  input->command latency: mean %.1f us, max %.1f us\n",
              s.meanLatencyMicros, s.maxLatencyMicros);
  std::printf("  worst robot-thread call: %.1f us (SetInputs %.1f, DispatchCommands %.1f)\n",
              worstCallMicros, worstSetMicros, worstDispatchMicros);
  std::printf("  mean robot-thread call: SetInputs %.2f us, DispatchCommands %.2f us\n",
              totalSetMicros / kTicks, totalDispatchMicros / kTicks);
  std::printf("  RSS: %ld kB at 30 s -> %ld kB at end (%+ld kB)\n", rssWarm, rssEnd, rssEnd - rssWarm);

  CHECK(s.inputsSubmitted == kTicks);
  CHECK(s.inputsProcessed + s.inputsOverwritten == kTicks);
  // At the real 20 ms period Soar must keep up with every loop. With the
  // period compressed to 1 ms an occasional scheduler hiccup may skip one.
  CHECK(s.inputsOverwritten <= (realtime ? 0 : kTicks / 100));
  CHECK(s.commandsRejected == 0);
  CHECK(s.commandsDropped == 0);
  CHECK(rec.moves > 100);
  CHECK(rec.shots > 10);
  CHECK(s.commandsReceived == rec.moves + rec.intakes + rec.shots);
  std::printf("  inputs over the 5 ms latency budget: %lld\n", s.latencyOver5ms);
  CHECK(s.meanLatencyMicros < 5000.0);   // spec: I/O latency < 5 ms
  // The compressed run sleeps 15000 times on a non-real-time OS, so allow the
  // scheduler a 0.1% outlier rate there; the real-time run allows none.
  CHECK(s.latencyOver5ms <= (realtime ? 0 : kTicks / 1000));
  CHECK(worstCallMicros < 1000.0);       // robot thread never waits on Soar
  CHECK(rssEnd - rssWarm < 1024);        // no growth over the match
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::printf("usage: %s <agents dir> <tests dir> [--realtime]\n", argv[0]);
    return 2;
  }
  std::string agents = argv[1], tests = argv[2];
  bool realtime = argc > 3 && !std::strcmp(argv[3], "--realtime");

  TestStartFailure();
  TestMapping(agents);
  TestBadCommands(tests);
  TestGenericIO(tests);
  TestDropped(tests);
  TestRestart(agents);
  TestMatch(agents, realtime);

  std::printf(g_failures ? "FAILED (%d checks)\n" : "PASSED\n", g_failures);
  return g_failures ? 1 : 0;
}
