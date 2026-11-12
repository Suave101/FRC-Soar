# Telemetry and debugging

## Statistics

`GetStats()` (Python `get_stats()`) returns counters and timings. Publish them
to the dashboard to watch the agent on the real robot.

| Field (C++) | Python | Meaning |
| --- | --- | --- |
| `inputsSubmitted` | `inputs_submitted` | `SendInputs()` / `SetInputs()` calls |
| `inputsProcessed` | `inputs_processed` | Snapshots that reached the input-link |
| `inputsOverwritten` | `inputs_overwritten` | Snapshots merged into a newer one before Soar read them |
| `inputErrors` | `input_errors` | Setter calls refused (bad path or conflict) |
| `decisionCycles` | `decision_cycles` | Decision cycles run |
| `commandsReceived` | `commands_received` | Valid commands read from the output-link |
| `commandsRejected` | `commands_rejected` | Unknown name, or missing or invalid parameter |
| `commandsDropped` | `commands_dropped` | Discarded because the robot did not dispatch them in time |
| `commandsUnhandled` | `commands_unhandled` | Dispatched with no handler for the name |
| `commandsCompleted` | `commands_completed` | `^status` written for dispatched commands |
| `outputConstantsIgnored` | `output_constants_ignored` | Constants put directly on the output-link |
| `lastLatencyMicros` | `last_latency_micros` | Latest `SendInputs()` → commands ready time, µs |
| `meanLatencyMicros` | `mean_latency_micros` | Mean of the above |
| `maxLatencyMicros` | `max_latency_micros` | Worst of the above |
| `latencyOver5ms` | `latency_over_5ms` | Inputs whose latency was over 5 ms |

Java exposes the most useful ones as methods on `SoarBridge`:
`meanLatencyMillis()`, `maxLatencyMillis()`, `inputsProcessed()`,
`commandsRejected()`, `commandsDropped()`, `commandsUnhandled()`,
`outputConstantsIgnored()` and `inputErrors()`.

```java
SmartDashboard.putNumber("Soar/mean latency ms", soar.meanLatencyMillis());
SmartDashboard.putNumber("Soar/max latency ms", soar.maxLatencyMillis());
SmartDashboard.putNumber("Soar/rejected", soar.commandsRejected());
SmartDashboard.putString("Soar/warning", soar.lastWarning());
```

### What to look for

- **`inputsProcessed` much lower than `inputsSubmitted`**, or a rising
  `inputsOverwritten`: the agent takes longer than a loop. Simplify the rules,
  lower `decisionsPerInput`, or set `realtimePriority`.
- **`latencyOver5ms` rising:** commands may miss the loop they were meant for.
- **`commandsRejected` rising:** a production uses a name you did not register,
  or a built-in command is missing a parameter. Check `GetLastWarning()`.
- **`commandsDropped` above zero:** the robot is not calling
  `DispatchCommands()` often enough.
- **`commandsUnhandled` above zero:** a name is accepted but no handler is
  registered for it.

## Warnings

`GetLastWarning()` (Java `lastWarning()`, Python `get_last_warning()`) holds the
most recent problem that did not stop the bridge, in words:

- a refused input path, and why;
- a rejected command, and why;
- a dispatched command with no handler;
- a constant on the output-link, with the identifier form to use instead;
- commands dropped because the queue was full.

It is empty if nothing has gone wrong.

## Attaching the Soar debugger

Set `debuggerPort` to a TCP port to let the Soar Java debugger connect to the
running agent. Use a remote connection to the robot's address and that port.

=== "Java"

    ```java
    // productions, decisionsPerInput, debuggerPort
    soar = new SoarBridge(path, 1, 12121);
    ```

=== "C++"

    ```cpp
    soarbridge::SoarHardwareInterface soar{{.productionsPath = path, .debuggerPort = 12121}};
    ```

=== "Python"

    ```python
    soar = soarbridge.SoarHardwareInterface(productions_path=path, debugger_port=12121)
    ```

While a debugger port is set, the Soar thread also wakes every 50 ms to service
the debugger, even when no inputs arrive. Leave it at `0` (off) for
competition.

!!! warning
    The bridge sets `trace 0` at startup. Turn tracing back on from the
    debugger if you want to see it, and expect it to slow the agent down.
