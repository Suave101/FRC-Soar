# Soar I/O Mapping Specification

Working-memory elements exchanged between the WPILib robot program and the Soar
agent. Paths are relative to `^io.input-link` / `^io.output-link`; a dotted path
such as `^match.time` is an identifier `^match` with an attribute `^time`.

Soar has no boolean type. Booleans are the string constants `true` and `false`.

## 1. Input-link (robot -> Soar)

Written from the `RobotInputs` snapshot passed to `SetInputs()` once per 20 ms
loop. A WME whose value did not change is left untouched (it does not blink).

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

## 2. Output-link (Soar -> robot)

Each command is an identifier on the output-link with the parameters below.
All listed parameters are required.

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
accepts `1` / `0`.

### Command lifecycle

1. The agent creates the command on the output-link.
2. The bridge reads it at the end of that decision cycle and queues it for the
   robot thread.
3. The robot thread executes it (`DispatchCommands`, or `PollCommands` +
   `CompleteCommand`).
4. The bridge adds `^status complete` to the command. The agent sees it on its
   next decision cycle and should remove the command.

A command is delivered to the robot exactly once, however long it stays on the
output-link. The bridge adds `^status error` instead, and does not forward the
command, when the command name is not in the table above or a parameter is
missing or not of the listed type. `^status error` is also used when the robot
code's handler throws.

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

`agents/default.soar` is a complete working example.
