// Plain-data types exchanged between the robot loop and the Soar thread.
// See docs/io-mapping.md for the WME each field maps to.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

namespace soarbridge {

enum class MatchMode { Disabled, Autonomous, Teleop, Test };
enum class Alliance { Red, Blue };

// Soar constant types. Soar has no boolean: booleans are the strings
// "true" / "false". Int and Float are different symbols to Soar, so a
// production testing `^beam-break 1` matches an Int 1 but not a Float 1.0.
enum class ValueType { Int, Float, String };

// One input-link value or one command parameter.
struct Value {
  ValueType type = ValueType::String;
  long long intValue = 0;
  double floatValue = 0.0;
  std::string stringValue;

  static Value FromInt(long long v) { Value r; r.type = ValueType::Int; r.intValue = v; return r; }
  static Value FromFloat(double v) { Value r; r.type = ValueType::Float; r.floatValue = v; return r; }
  static Value FromString(const std::string& v) { Value r; r.stringValue = v; return r; }
  static Value FromBool(bool v) { return FromString(v ? "true" : "false"); }

  // Int or Float, or a String that is entirely a number.
  bool AsNumber(double* out) const {
    if (type == ValueType::Int) { *out = static_cast<double>(intValue); return true; }
    if (type == ValueType::Float) { *out = floatValue; return true; }
    const char* text = stringValue.c_str();
    if (*text == '\0') return false;
    char* end = nullptr;
    double v = std::strtod(text, &end);
    if (end == text || *end != '\0') return false;
    *out = v;
    return true;
  }
  // "true" / "false", or the integer (or string) 1 / 0.
  bool AsBool(bool* out) const {
    if (type == ValueType::Int && (intValue == 0 || intValue == 1)) { *out = intValue == 1; return true; }
    if (type != ValueType::String) return false;
    if (stringValue == "true" || stringValue == "1") { *out = true; return true; }
    if (stringValue == "false" || stringValue == "0") { *out = false; return true; }
    return false;
  }
  // The value as Soar would print it.
  std::string ToString() const {
    if (type == ValueType::Int) return std::to_string(intValue);
    if (type == ValueType::Float) return std::to_string(floatValue);
    return stringValue;
  }
  bool operator==(const Value& o) const {
    if (type != o.type) return false;
    if (type == ValueType::Int) return intValue == o.intValue;
    if (type == ValueType::Float) return floatValue == o.floatValue;
    return stringValue == o.stringValue;
  }
  bool operator!=(const Value& o) const { return !(*this == o); }
};

// Typed snapshot of the fixed schema in docs/io-mapping.md section 1. It is a
// convenience layer: SetInputs(RobotInputs) writes these fields through the
// same path setters (SetNumber("match.time", ...)) robot code can call itself.
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

// The five built-in commands, plus Custom for any other accepted name.
enum class CommandType { Move, TurnTo, SetElevator, RunIntake, Shoot, Custom };

// One command read from the Soar output-link: an identifier `^<name> <cmd>`
// whose children are the parameters. Nested identifiers are flattened into
// dotted keys, so `(<cmd> ^target <t>) (<t> ^x 1.5)` is params["target.x"].
//
// For the built-in commands `type` and the typed fields below are filled in
// as well; for every other command `type` is Custom and they are zero.
struct Command {
  CommandType type = CommandType::Custom;
  // Identifies the output-link command; pass to CompleteCommand().
  long long id = 0;
  std::string name;                       // output-link attribute, e.g. "ground-intake"
  std::map<std::string, Value> params;    // every parameter except ^status

  double vx = 0.0;        // move (m/s)
  double vy = 0.0;        // move (m/s)
  double omega = 0.0;     // move (rad/s)
  double heading = 0.0;   // turn-to (deg, absolute)
  double height = 0.0;    // set-elevator (m)
  double speed = 0.0;     // run-intake (-1..1)
  bool execute = false;   // shoot

  bool Has(const std::string& key) const { return params.count(key) != 0; }
  // Missing keys return a String value "".
  Value GetParam(const std::string& key) const {
    auto it = params.find(key);
    return it == params.end() ? Value() : it->second;
  }
  double GetNumber(const std::string& key, double fallback = 0.0) const {
    auto it = params.find(key);
    double v;
    return it != params.end() && it->second.AsNumber(&v) ? v : fallback;
  }
  std::string GetString(const std::string& key, const std::string& fallback = "") const {
    auto it = params.find(key);
    return it == params.end() ? fallback : it->second.ToString();
  }
  bool GetBool(const std::string& key, bool fallback = false) const {
    auto it = params.find(key);
    bool v;
    return it != params.end() && it->second.AsBool(&v) ? v : fallback;
  }
  std::vector<std::string> ParamNames() const {
    std::vector<std::string> names;
    names.reserve(params.size());
    for (const auto& p : params) names.push_back(p.first);
    return names;
  }
};

// Receives the built-in commands on the robot thread. Subclass and forward to
// subsystems. Other commands go to handlers registered with OnCommand().
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
  long long inputsSubmitted = 0;    // SendInputs() / SetInputs() calls
  long long inputsProcessed = 0;    // snapshots that reached the input-link
  long long inputsOverwritten = 0;  // snapshots replaced before Soar read them
  long long inputErrors = 0;        // Set*/RemoveInput calls refused (bad path, conflict)
  long long decisionCycles = 0;
  long long commandsReceived = 0;   // valid commands read from the output-link
  long long commandsRejected = 0;   // unknown name or missing/invalid parameter
  long long commandsDropped = 0;    // discarded because the robot never polled (^status error)
  long long commandsUnhandled = 0;  // dispatched with no handler for the name (^status error)
  long long commandsCompleted = 0;  // ^status WMEs written for polled commands
  long long outputConstantsIgnored = 0;  // non-identifier WMEs on the output-link
  // SendInputs() -> resulting commands available to PollCommands(), microseconds.
  double lastLatencyMicros = 0.0;
  double maxLatencyMicros = 0.0;
  double meanLatencyMicros = 0.0;
  long long latencyOver5ms = 0;     // inputs whose latency exceeded the 5 ms budget
};

}  // namespace soarbridge
