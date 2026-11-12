# Python API

Module `soarbridge`, a compiled extension for RobotPy. Names follow Python
conventions (`send_inputs` for C++ `SendInputs`), and behaviour matches the
[C++ API](cpp.md).

## `SoarHardwareInterface`

```python
soarbridge.SoarHardwareInterface(
    productions_path="", agent_name="robot", decisions_per_input=1,
    debugger_port=0, max_pending_commands=256, realtime_priority=0)
```

See [Configuration](configuration.md).

| Method | Description |
| --- | --- |
| `start()` | Starts Soar and loads the productions. Raises `RuntimeError` on failure |
| `stop()` | Stops Soar. Safe to call repeatedly |
| `is_running()` | |
| `set_number(path, value)` | Float WME. Returns `False` if the path is refused |
| `set_int(path, value)` | Int WME |
| `set_string(path, value)` | String WME |
| `set_bool(path, value)` | The string `true` or `false` |
| `remove_input(path)` | Removes a value, or an identifier and everything under it |
| `send_inputs()` | Publishes the values staged since the last call |
| `set_inputs(inputs)` | Writes a `RobotInputs` to its fixed paths, then `send_inputs()` |
| `on_command(name, handler)` | Accepts commands named `name` and calls `handler(command)` for each in `dispatch_commands()` |
| `accept_command(name)` | Accepts the name without a handler, for `poll_commands()` |
| `dispatch_commands(handler=None)` | Runs `on_command` handlers, and for built-in commands, `handler` (a `CommandHandler`). Returns the number polled |
| `poll_commands()` | List of new `Command`s. Complete or reject each one |
| `complete_command(command_id, success=True)` | `^status complete` or `error` |
| `reject_command(command_id, reason)` | `^status error`, with `reason` in the warning |
| `get_stats()` | `BridgeStats` |
| `get_last_warning()` | Most recent non-fatal problem, or `""` |

It is also a context manager: `with` calls `start()` on entry and `stop()` on
exit.

```python
with soarbridge.SoarHardwareInterface(productions_path="robot.soar") as soar:
    soar.set_bool("ground-intake.beam-break", True)
    soar.send_inputs()
```

`start()` and `stop()` release the GIL while they wait. Handlers run with the
GIL held, on the thread that calls `dispatch_commands()`.

## `Command`

| Attribute / method | Description |
| --- | --- |
| `name` | Output-link attribute |
| `id` | Pass to `complete_command` / `reject_command` |
| `type` | `CommandType.MOVE`, `TURN_TO`, `SET_ELEVATOR`, `RUN_INTAKE`, `SHOOT` or `CUSTOM` |
| `params` | `dict` of every parameter except `^status`. Values are `int`, `float` or `str` |
| `has(key)` | |
| `get_number(key, default=0.0)` | Int, float, or numeric string |
| `get_string(key, default="")` | Any value as Soar prints it |
| `get_bool(key, default=False)` | `true` / `false`, or `1` / `0` |
| `vx`, `vy`, `omega`, `heading`, `height`, `speed`, `execute` | Typed parameters, for built-in commands only |

## `CommandHandler`

Subclass it and override `move(vx, vy, omega)`, `turn_to(heading)`,
`set_elevator(height)`, `run_intake(speed)` and `shoot(execute)`. Call
`soarbridge.CommandHandler.__init__(self)` in your constructor (or
`robotInit`).

## `RobotInputs`

Fields: `match_time`, `mode` (`MatchMode.DISABLED`, `AUTONOMOUS`, `TELEOP`,
`TEST`), `alliance` (`Alliance.RED`, `BLUE`), `pose_x`, `pose_y`, `heading`,
`velocity_x`, `velocity_y`, `target_detected`, `target_distance`,
`target_yaw`, `has_piece`, `elevator_height`. See
[The fixed schema](../guide/fixed-schema.md#inputs).

## `BridgeStats`

Read-only fields: `inputs_submitted`, `inputs_processed`,
`inputs_overwritten`, `input_errors`, `decision_cycles`, `commands_received`,
`commands_rejected`, `commands_dropped`, `commands_unhandled`,
`commands_completed`, `output_constants_ignored`, `last_latency_micros`,
`max_latency_micros`, `mean_latency_micros`, `latency_over_5ms`. See
[Telemetry](../guide/telemetry.md#statistics).
