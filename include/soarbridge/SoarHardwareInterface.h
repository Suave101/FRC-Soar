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
//
// Any robot, by path (no fixed schema):
//
//   soar.OnCommand("ground-intake", [&](const soarbridge::Command& c) {
//     intake.SetState(c.GetString("state"));
//   });                                // RobotInit
//   soar.SetBool("ground-intake.beam-break", !beamBreak.Get());
//   soar.SetString("match.mode", "teleop");
//   soar.SendInputs();                 // start of RobotPeriodic
//   soar.DispatchCommands();           // later in the same loop
#pragma once

#include <memory>
#include <string>
#include <vector>
#ifndef SWIG
#include <functional>
#endif

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

  // Input-link values by path, relative to ^io.input-link: "a.b.c" is
  // ^a <x>, <x> ^b <y>, <y> ^c value. Missing identifiers are created on
  // demand. Each segment starts with a letter and contains only letters,
  // digits, '-' and '_'. A path cannot be both a value and a parent of other
  // values. Returns false (and counts BridgeStats::inputErrors, sets
  // GetLastWarning()) for an invalid or conflicting path.
  //
  // Values are staged until SendInputs(), so the agent always sees one
  // loop's values together. An unchanged value is not re-sent.
  bool SetNumber(const std::string& path, double value);       // float WME
  bool SetInt(const std::string& path, long long value);        // int WME
  bool SetString(const std::string& path, const std::string& value);
  bool SetBool(const std::string& path, bool value);            // "true"/"false"
  // Removes a value, or an identifier and everything under it. Removing the
  // last child of an identifier leaves the (empty) identifier in place.
  // False only for an invalid path.
  bool RemoveInput(const std::string& path);

  // Publishes everything staged since the last call as one snapshot and lets
  // the agent run. If Soar has not consumed the previous snapshot yet the two
  // are merged; the agent always sees the newest value of every path.
  void SendInputs();

  // Typed layer over the path setters: writes every RobotInputs field to its
  // path (docs/io-mapping.md section 1), then SendInputs().
  void SetInputs(const RobotInputs& inputs);

  // Lets the agent issue output-link commands named `name`. Other names are
  // rejected with ^status error. The built-in commands (move, turn-to,
  // set-elevator, run-intake, shoot) are always accepted. Thread-safe. False
  // if the name is not a valid single path segment.
  bool AcceptCommand(const std::string& name);

#ifndef SWIG
  using CommandCallback = std::function<void(const Command&)>;
  // AcceptCommand(name) plus a handler that DispatchCommands() calls with
  // each such command. Replaces an earlier handler for the same name. Call
  // from the robot thread (normally RobotInit), not from inside a handler.
  bool OnCommand(const std::string& name, CommandCallback handler);
#endif

  // Takes all commands the agent has issued since the last call.
  std::vector<Command> PollCommands();

  // Marks a polled command as executed: Soar adds ^status complete (or
  // ^status error if success is false) to it on the output-link.
  void CompleteCommand(long long commandId, bool success = true);

  // Marks a polled command that robot code cannot handle: ^status error,
  // BridgeStats::commandsUnhandled, and `reason` in GetLastWarning().
  void RejectCommand(long long commandId, const std::string& reason);

  // PollCommands, then for each command: its OnCommand handler, or (for the
  // built-in commands only) the matching CommandHandler method, then
  // CompleteCommand. A command with neither is passed to RejectCommand.
  // Returns the number of commands polled. If a handler throws, that command
  // is completed with ^status error, the rest are still dispatched, and the
  // first exception is rethrown.
  int DispatchCommands(CommandHandler& handler);
  int DispatchCommands();

  BridgeStats GetStats() const;

  // Most recent problem that did not stop the bridge: a refused input path,
  // a rejected or unhandled command, or a constant WME on the output-link
  // (commands must be identifiers). Empty if none.
  std::string GetLastWarning() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace soarbridge
