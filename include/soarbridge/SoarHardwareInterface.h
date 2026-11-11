// Bridge between a WPILib robot loop and a Soar agent.
//
// The Soar kernel and agent live entirely on a background thread. The robot
// thread only ever copies a small struct in or swaps a vector out under a
// mutex that the Soar thread holds for the same kind of copy, never while
// Soar is running, so calls from the 20 ms loop return in well under a
// microsecond regardless of what the agent is doing.
//
// Typical use:
//
//   soarbridge::SoarHardwareInterface soar{{.productionsPath = "/home/lvuser/deploy/robot.soar"}};
//   soar.Start();                      // RobotInit
//   ...
//   soar.SetInputs(inputs);            // start of RobotPeriodic
//   soar.DispatchCommands(handler);    // later in the same loop
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "soarbridge/RobotIO.h"

namespace soarbridge {

struct BridgeConfig {
  // .soar file sourced at startup. May contain any Soar CLI commands.
  std::string productionsPath;
  std::string agentName = "robot";
  // Decision cycles run for each input snapshot.
  int decisionsPerInput = 1;
  // TCP port for attaching the Soar debugger; 0 disables the listener.
  int debuggerPort = 0;
  // Commands held for the robot thread before the oldest are dropped.
  int maxPendingCommands = 256;
  // 0 leaves the Soar thread at normal priority. 1-99 requests SCHED_FIFO at
  // that priority (Linux), which keeps latency bounded when other threads
  // compete for the CPU. Use a value below the robot loop's own priority.
  // Ignored if the process lacks permission.
  int realtimePriority = 0;
};

class SoarHardwareInterface {
 public:
  explicit SoarHardwareInterface(const BridgeConfig& config = BridgeConfig());
  ~SoarHardwareInterface();

  SoarHardwareInterface(const SoarHardwareInterface&) = delete;
  SoarHardwareInterface& operator=(const SoarHardwareInterface&) = delete;

  // Starts the Soar thread and blocks until the kernel is up and productions
  // are loaded (tens of ms; call from RobotInit, not the periodic loop).
  // Returns false and sets GetLastError() on failure.
  bool Start();

  // Stops the thread and frees the kernel. Safe to call repeatedly.
  void Stop();

  bool IsRunning() const;
  std::string GetLastError() const;

  // --- Robot thread, every loop. None of these block on Soar. ---

  // Publishes a snapshot for the input-link. If Soar has not consumed the
  // previous snapshot yet it is replaced; the agent always sees the newest.
  void SetInputs(const RobotInputs& inputs);

  // Takes all commands the agent has issued since the last call.
  std::vector<Command> PollCommands();

  // Marks a polled command as executed: Soar adds ^status complete (or
  // ^status error if success is false) to it on the output-link.
  void CompleteCommand(long long commandId, bool success = true);

  // PollCommands + call the matching handler method + CompleteCommand for
  // each. Returns the number of commands dispatched. If a handler throws, that
  // command is completed with ^status error, the rest are still dispatched,
  // and the first exception is rethrown.
  int DispatchCommands(CommandHandler& handler);

  BridgeStats GetStats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace soarbridge
