# C++ API

Namespace `soarbridge`. Headers:

- `soarbridge/SoarHardwareInterface.h`: the bridge and `BridgeConfig`.
- `soarbridge/RobotIO.h`: data types (included by the above).
- `soarbridge/WpilibAdapter.h`: optional WPILib helpers. See
  [The fixed schema](../guide/fixed-schema.md#wpilib-helpers-c).

The library needs C++17. Link `libsoarbridge.so`.

## `SoarHardwareInterface`

Owns the Soar thread. Not copyable.

### Lifecycle

| Method | Description |
| --- | --- |
| `explicit SoarHardwareInterface(const BridgeConfig& = BridgeConfig())` | Stores the configuration. Does not start Soar |
| `bool Start()` | Starts the Soar thread and blocks until the kernel is up and the productions are loaded (tens of ms). Call from `RobotInit`. Returns `false` and sets `GetLastError()` on failure |
| `void Stop()` | Stops the thread and frees the kernel. Safe to call repeatedly. Also called by the destructor |
| `bool IsRunning() const` | Whether the Soar thread is running |
| `std::string GetLastError() const` | Why `Start()` failed |

### Inputs

None of these wait on Soar.

| Method | Description |
| --- | --- |
| `bool SetNumber(const std::string& path, double value)` | Stages a float WME |
| `bool SetInt(const std::string& path, long long value)` | Stages an int WME |
| `bool SetString(const std::string& path, const std::string& value)` | Stages a string WME |
| `bool SetBool(const std::string& path, bool value)` | Stages the string `true` or `false` |
| `bool RemoveInput(const std::string& path)` | Removes a value, or an identifier and everything under it. `false` only for an invalid path |
| `void SendInputs()` | Publishes everything staged since the last call as one snapshot and lets the agent run |
| `void SetInputs(const RobotInputs& inputs)` | Writes every `RobotInputs` field to its path, then `SendInputs()` |

The setters return `false` for an invalid or conflicting path, add to
`BridgeStats::inputErrors`, and set `GetLastWarning()`. See
[Inputs by path](../guide/inputs.md#path-rules).

### Commands

| Method | Description |
| --- | --- |
| `bool OnCommand(const std::string& name, CommandCallback handler)` | Accepts commands named `name` and calls `handler` for each in `DispatchCommands()`. Replaces an earlier handler for the name. Call from the robot thread, not from inside a handler. `false` for an invalid name |
| `bool AcceptCommand(const std::string& name)` | Accepts the name without a handler, for use with `PollCommands()`. Thread-safe |
| `int DispatchCommands()` | Polls, then calls each command's `OnCommand` handler and completes it. Returns the number polled |
| `int DispatchCommands(CommandHandler& handler)` | As above. Built-in commands with no `OnCommand` handler go to `handler` |
| `std::vector<Command> PollCommands()` | Takes all commands issued since the last call. You must complete or reject each one |
| `void CompleteCommand(long long commandId, bool success = true)` | Writes `^status complete` (or `error` if `success` is false) |
| `void RejectCommand(long long commandId, const std::string& reason)` | Writes `^status error`, counts `commandsUnhandled`, and puts `reason` in `GetLastWarning()` |

`CommandCallback` is `std::function<void(const Command&)>`.

If a handler throws during `DispatchCommands`, that command gets
`^status error`, the remaining commands are still dispatched, and then the
first exception is rethrown. A command with no handler is passed to
`RejectCommand`.

### Diagnostics

| Method | Description |
| --- | --- |
| `BridgeStats GetStats() const` | Counters and latency. See [Telemetry](../guide/telemetry.md#statistics) |
| `std::string GetLastWarning() const` | Most recent non-fatal problem, or empty |

## `BridgeConfig`

See [Configuration](configuration.md).

## `RobotInputs`

Plain struct for the fixed schema. Fields and paths are listed in
[The fixed schema](../guide/fixed-schema.md#inputs).

```cpp
enum class MatchMode { Disabled, Autonomous, Teleop, Test };
enum class Alliance { Red, Blue };
```

## `Command`

| Member | Description |
| --- | --- |
| `CommandType type` | `Move`, `TurnTo`, `SetElevator`, `RunIntake`, `Shoot`, or `Custom` for any other name |
| `long long id` | Identifies the command. Pass to `CompleteCommand` / `RejectCommand` |
| `std::string name` | Output-link attribute, e.g. `"ground-intake"` |
| `std::map<std::string, Value> params` | Every parameter except `^status`. Nested identifiers use dotted keys |
| `double vx, vy, omega, heading, height, speed`; `bool execute` | Typed parameters, filled for built-in commands only |
| `bool Has(key) const` | Whether a parameter is present |
| `Value GetParam(key) const` | The raw value. Missing keys return an empty string `Value` |
| `double GetNumber(key, fallback = 0.0) const` | Int, float, or numeric string |
| `std::string GetString(key, fallback = "") const` | Any value, as Soar prints it |
| `bool GetBool(key, fallback = false) const` | `true` / `false`, or `1` / `0` |
| `std::vector<std::string> ParamNames() const` | All parameter keys |

## `Value`

One input value or command parameter.

| Member | Description |
| --- | --- |
| `ValueType type` | `Int`, `Float` or `String` |
| `long long intValue`, `double floatValue`, `std::string stringValue` | The value, by type |
| `static Value FromInt / FromFloat / FromString / FromBool(v)` | Constructors. `FromBool` gives the string `true` / `false` |
| `bool AsNumber(double* out) const` | Int, float, or a string that is entirely a number |
| `bool AsBool(bool* out) const` | `true` / `false`, or `1` / `0` |
| `std::string ToString() const` | As Soar would print it |

## `CommandHandler`

Abstract interface for the built-in commands:

```cpp
class CommandHandler {
 public:
  virtual void Move(double vx, double vy, double omega) = 0;
  virtual void TurnTo(double headingDegrees) = 0;
  virtual void SetElevator(double heightMeters) = 0;
  virtual void RunIntake(double speed) = 0;
  virtual void Shoot(bool execute) = 0;
};
```

## `BridgeStats`

See [Telemetry and debugging](../guide/telemetry.md#statistics).
