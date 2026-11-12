# Soar I/O Mapping Specification

Working-memory elements exchanged between the WPILib robot program and the Soar
agent. Paths are relative to `^io.input-link` / `^io.output-link`; a dotted path
such as `^match.time` is an identifier `^match` with an attribute `^time`.

Soar has no boolean type. Booleans are the string constants `true` and `false`.

The bridge has two layers:

| Layer | Inputs | Commands | Use it when |
| --- | --- | --- | --- |
| Generic | `SetNumber` / `SetInt` / `SetString` / `SetBool` by path, then `SendInputs()` | Any identifier on the output-link, delivered as `name` + `params`; handlers registered by name | Your robot has its own subsystems (any robot, e.g. one designed in SysMod Studio) |
| Typed | `RobotInputs` struct, `SetInputs()` | `move`, `turn-to`, `set-elevator`, `run-intake`, `shoot` via `CommandHandler` | The fixed schema in sections 2 and 4 fits |

The typed layer is built on the generic one: `SetInputs()` writes each
`RobotInputs` field to its path, and the five built-in commands are ordinary
commands whose parameters are also copied into typed fields. Both can be used
at once.

## 1. Input-link by path (robot -> Soar)

| C++ | Java (`SoarBridge`) | Python | WME created |
| --- | --- | --- | --- |
| `SetNumber("arm.angle", 30.5)` | `setNumber(...)` | `set_number(...)` | `^arm.angle 30.5` (float) |
| `SetInt("ground-intake.beam-break", 1)` | `setInt(...)` | `set_int(...)` | `^ground-intake.beam-break 1` (int) |
| `SetString("match.mode", "teleop")` | `setString(...)` | `set_string(...)` | `^match.mode teleop` |
| `SetBool("ground-intake.has-piece", true)` | `setBool(...)` | `set_bool(...)` | `^ground-intake.has-piece true` |
| `RemoveInput("vision")` | `removeInput(...)` | `remove_input(...)` | removes `^vision` and everything under it |
| `SendInputs()` | `sendInputs()` | `send_inputs()` | publishes the changes above, runs the agent |

Rules:

- **Path syntax.** Dot-separated names. Each name starts with a letter and uses
  only letters, digits, `-` and `_`. No leading `^`.
- **Parents are created on demand.** `SetNumber("a.b.c", 1)` creates `^a` and
  `^b` identifiers if they do not exist.
- **A path is a value or a parent, not both.** After `SetNumber("arm.angle", 1)`,
  `SetNumber("arm.angle.x", 2)` and `SetNumber("arm", 2)` are refused. Remove
  the old path first.
- **Refused calls** return `false`, add one to `BridgeStats::inputErrors`, and
  say why in `GetLastWarning()`. Nothing is sent.
- **Int vs float matters.** Soar treats `1` and `1.0` as different constants. A
  production testing `^beam-break 1` matches `SetInt(..., 1)` but not
  `SetNumber(..., 1.0)`. Comparisons such as `^angle > 10` work for both.
- **Snapshots.** Values are staged until `SendInputs()`, so the agent sees one
  loop's values together. If Soar has not used the previous snapshot yet, the
  two are merged and the newest value of each path wins.
- **Unchanged values are not re-sent** and do not blink.
- **Changing type** (for example string to float) replaces the WME.
- **Removing** a value leaves its parent identifier, even if empty. Remove the
  parent to remove the whole group.
- **Restart.** After `Stop()` / `Start()` every current value is sent again to
  the new agent at the next `SendInputs()`.

The robot thread never waits on Soar. Setters touch only robot-side state;
`SendInputs()` hands the changes over with one short lock.

## 2. Fixed input schema (`RobotInputs`)

`SetInputs(inputs)` writes these paths with the generic setters, then calls
`SendInputs()`. The paths appear the first time `SetInputs()` is called. Robots
that only use the generic setters never see them.

| WME | Type | `RobotInputs` field | Meaning |
| --- | --- | --- | --- |
| `^match.time` | float | `matchTime` | Match time remaining, seconds |
| `^match.mode` | string | `mode` | `disabled`, `autonomous`, `teleop` or `test` |
| `^match.alliance` | string | `alliance` | `red` or `blue` |
| `^drive.pose.x` | float | `poseX` | X position, meters |
| `^drive.pose.y` | float | `poseY` | Y position, meters |
| `^drive.pose.heading` | float | `heading` | Gyro heading, degrees |
| `^drive.velocity.vx` | float | `velocityX` | X velocity, m/s |
| `^drive.velocity.vy` | float | `velocityY` | Y velocity, m/s |
| `^vision.target-detected` | boolean | `targetDetected` | Primary AprilTag/Note in view |
| `^vision.target-distance` | float | `targetDistance` | Distance to target, meters |
| `^vision.target-yaw` | float | `targetYaw` | Angle offset to target, degrees |
| `^intake.has-piece` | boolean | `hasPiece` | Beam break / color sensor sees a piece |
| `^elevator.height` | float | `elevatorHeight` | Elevator extension, meters |

You can add your own paths next to these, e.g. `SetNumber("intake.current", 12.5)`.
You cannot turn one of them into a parent or a value (e.g. `SetNumber("match", 1)`
is refused once `SetInputs()` has been used).

## 3. Commands (Soar -> robot)

A command is an **identifier** on the output-link. Its name is the attribute;
its parameters are the identifier's children:

```soar
(<ol> ^ground-intake <c>)
(<c> ^state closed ^speed 0.5 ^target <t>)
(<t> ^x 1.5 ^y -2)
```

arrives as

| Field | Value |
| --- | --- |
| `name` | `"ground-intake"` |
| `id` | the command's time tag (pass to `CompleteCommand`) |
| `params` | `state` -> `"closed"`, `speed` -> `0.5`, `target.x` -> `1.5`, `target.y` -> `-2` |

- Nested identifiers become dotted keys, up to 4 levels deep.
- Each value keeps its Soar type: int, float or string (C++ `Value`; Java
  `Long` / `Double` / `String`; Python `int` / `float` / `str`).
- `^status` is not a parameter; the bridge writes it.
- Helpers: `GetNumber` / `GetString` / `GetBool` / `Has` (C++), `getDouble` /
  `getString` / `getBoolean` / `has` (Java `SoarCommand`), `get_number` /
  `get_string` / `get_bool` / `has` and `params` dict (Python).

### Registering command names

The bridge only accepts names it has been told about. Anything else is
rejected (see section 5), so a typo in a production shows up as an error
instead of being silently ignored.

| | C++ | Java | Python |
| --- | --- | --- | --- |
| Accept a name and handle it | `OnCommand(name, fn)` | `onCommand(name, listener)` | `on_command(name, fn)` |
| Accept a name only (you poll) | `AcceptCommand(name)` | — | `accept_command(name)` |
| Run handlers | `DispatchCommands()` | `dispatchCommands()` | `dispatch_commands()` |
| Handlers + built-ins | `DispatchCommands(handler)` | `dispatchCommands(actuators)` | `dispatch_commands(handler)` |
| Poll by hand | `PollCommands()` | — | `poll_commands()` |

Register handlers in robot init. Accepted names survive `Stop()` / `Start()`.

Dispatch calls, for each new command: the handler registered for its name;
otherwise, for a built-in command, the matching `CommandHandler` method;
otherwise nothing, and the command is marked `^status error` and counted in
`commandsUnhandled`.

## 4. Built-in commands

Always accepted. All listed parameters are required; the bridge rejects the
command if one is missing or not of the listed type. A handler registered with
`OnCommand` for one of these names takes precedence over `CommandHandler`.

| Command | Parameter | Type | Handler call |
| --- | --- | --- | --- |
| `^move` | `^vx` | float, m/s | `Move(vx, vy, omega)` |
| | `^vy` | float, m/s | |
| | `^omega` | float, rad/s | |
| `^turn-to` | `^heading` | float, degrees (absolute) | `TurnTo(heading)` |
| `^set-elevator` | `^height` | float, meters | `SetElevator(height)` |
| `^run-intake` | `^speed` | float, -1.0 to 1.0 | `RunIntake(speed)` |
| `^shoot` | `^execute` | boolean | `Shoot(execute)` |

Numeric parameters may be written as integers (`^speed 1`). `^execute` also
accepts `1` / `0`. These commands also have `type` set (`Move`, ...); every
other command has `type == Custom`.

## 5. Command lifecycle

1. The agent creates the command on the output-link.
2. The bridge reads it at the end of that decision cycle and queues it for the
   robot thread.
3. The robot thread executes it (`DispatchCommands`, or `PollCommands` +
   `CompleteCommand`).
4. The bridge adds `^status complete` to the command. The agent sees it on its
   next decision cycle and should remove the command.

A command is delivered to the robot exactly once, however long it stays on the
output-link. Every identifier command ends with a `^status`:

| Situation | `^status` | Counter (`BridgeStats`) | Delivered? |
| --- | --- | --- | --- |
| Handled | `complete` | `commandsCompleted` | yes |
| Handler threw, or `CompleteCommand(id, false)` | `error` | `commandsCompleted` | yes |
| Name not registered | `error` | `commandsRejected` | no |
| Built-in with a missing or invalid parameter | `error` | `commandsRejected` | no |
| A parameter given twice (`^v 1 ^v 2`), or nested more than 4 levels | `error` | `commandsRejected` | no |
| Dispatched but no handler for the name | `error` | `commandsUnhandled` | yes |
| More than `maxPendingCommands` waiting; oldest dropped | `error` | `commandsDropped` | no |

Dropped commands get `^status error` so the agent can clear and re-issue them;
otherwise a rule like "issue `^ping` if there is none" would wait on the
dropped one for ever.

**Constants on the output-link are not commands.** `(<ol> ^ground-intake closed)`
cannot carry a `^status`, so the bridge ignores it, counts it in
`outputConstantsIgnored` and explains in `GetLastWarning()`. Write it as an
identifier instead:

```soar
(<ol> ^ground-intake <c>)
(<c> ^state closed)
```

`GetLastWarning()` (Java `lastWarning()`, Python `get_last_warning()`) always
holds the most recent of these problems, in words.

Agents need a rule that removes finished commands, for example:

```soar
sp {robot*propose*clear-commands
   (state <s> ^superstate nil ^io.output-link <ol>)
   (<ol> ^<name> <cmd>)
   (<cmd> ^status)
-->
   (<s> ^operator <o> + > =)
   (<o> ^name clear-commands)}

sp {robot*apply*clear-commands
   (state <s> ^operator.name clear-commands ^io.output-link <ol>)
   (<ol> ^<name> <cmd>)
   (<cmd> ^status)
-->
   (<ol> ^<name> <cmd> -)}
```

`agents/default.soar` is a complete example of the typed layer;
`tests/generic_io.soar` exercises the generic one.

## 6. Example: a robot with its own subsystems

A robot with a "Ground Intake" subsystem (a beam break, and states `open` /
`closed`), in Java:

```java
// robotInit
soar.onCommand("ground-intake", c -> groundIntake.setState(c.getString("state")));

// robotPeriodic
soar.setString("match.mode", DriverStation.isTeleop() ? "teleop" : "disabled");
soar.setBool("ground-intake.beam-break", !beamBreak.get());
soar.sendInputs();
...
soar.dispatchCommands();
```

and the agent side:

```soar
sp {robot*apply*grab
   (state <s> ^operator.name grab ^io.output-link <ol>)
-->
   (<ol> ^ground-intake <c>)
   (<c> ^state closed)}
```
