// Plain-data types exchanged between the robot loop and the Soar thread.
// See docs/io-mapping.md for the WME each field maps to.
#pragma once

#include <cstdint>
#include <string>

namespace soarbridge {

enum class MatchMode { Disabled, Autonomous, Teleop, Test };
enum class Alliance { Red, Blue };

// Snapshot of robot state written to the Soar input-link. One per 20 ms loop.
struct RobotInputs {
  // ^match
  double matchTime = 0.0;                 // ^match.time (s remaining)
  MatchMode mode = MatchMode::Disabled;   // ^match.mode
  Alliance alliance = Alliance::Blue;     // ^match.alliance

  // ^drive
  double poseX = 0.0;        // ^drive.pose.x (m)
  double poseY = 0.0;        // ^drive.pose.y (m)
  double heading = 0.0;      // ^drive.pose.heading (deg)
  double velocityX = 0.0;    // ^drive.velocity.vx (m/s)
  double velocityY = 0.0;    // ^drive.velocity.vy (m/s)

  // ^vision
  bool targetDetected = false;   // ^vision.target-detected
  double targetDistance = 0.0;   // ^vision.target-distance (m)
  double targetYaw = 0.0;        // ^vision.target-yaw (deg)

  // mechanisms
  bool hasPiece = false;         // ^intake.has-piece
  double elevatorHeight = 0.0;   // ^elevator.height (m)
};

enum class CommandType { Move, TurnTo, SetElevator, RunIntake, Shoot };

// One command read from the Soar output-link. Only the fields belonging to
// `type` are meaningful; the rest are zero.
struct Command {
  CommandType type = CommandType::Move;
  // Identifies the output-link command; pass to CompleteCommand().
  long long id = 0;

  double vx = 0.0;        // move (m/s)
  double vy = 0.0;        // move (m/s)
  double omega = 0.0;     // move (rad/s)
  double heading = 0.0;   // turn-to (deg, absolute)
  double height = 0.0;    // set-elevator (m)
  double speed = 0.0;     // run-intake (-1..1)
  bool execute = false;   // shoot
};

// Receives commands on the robot thread. Subclass and forward to subsystems.
class CommandHandler {
 public:
  virtual ~CommandHandler() = default;
  virtual void Move(double vx, double vy, double omega) = 0;
  virtual void TurnTo(double headingDegrees) = 0;
  virtual void SetElevator(double heightMeters) = 0;
  virtual void RunIntake(double speed) = 0;
  virtual void Shoot(bool execute) = 0;
};

// Counters and timings, for telemetry and for checking the latency budget.
struct BridgeStats {
  long long inputsSubmitted = 0;    // SetInputs() calls
  long long inputsProcessed = 0;    // snapshots that reached the input-link
  long long inputsOverwritten = 0;  // snapshots replaced before Soar read them
  long long decisionCycles = 0;
  long long commandsReceived = 0;   // valid commands read from the output-link
  long long commandsRejected = 0;   // unknown name or missing/invalid parameter
  long long commandsDropped = 0;    // discarded because the robot never polled
  long long commandsCompleted = 0;  // ^status WMEs written
  // SetInputs() -> resulting commands available to PollCommands(), microseconds.
  double lastLatencyMicros = 0.0;
  double maxLatencyMicros = 0.0;
  double meanLatencyMicros = 0.0;
  long long latencyOver5ms = 0;     // inputs whose latency exceeded the 5 ms budget
};

}  // namespace soarbridge
