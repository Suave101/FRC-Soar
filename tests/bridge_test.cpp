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
  CHECK(all.size() == 1);
  if (!all.empty()) {
    CHECK(all[0].type == CommandType::SetElevator);
    CHECK(std::fabs(all[0].height - 0.5) < 1e-9);
  }
  BridgeStats s = soar.GetStats();
  CHECK(s.commandsReceived == 1);
  CHECK(s.commandsRejected == 2);
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
  double worstCallMicros = 0;
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
  std::printf("  worst robot-thread call: %.1f us\n", worstCallMicros);
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
  TestRestart(agents);
  TestMatch(agents, realtime);

  std::printf(g_failures ? "FAILED (%d checks)\n" : "PASSED\n", g_failures);
  return g_failures ? 1 : 0;
}
