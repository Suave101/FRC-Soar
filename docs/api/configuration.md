# Configuration

| C++ (`BridgeConfig`) | Python keyword | Java constructor | Default | Meaning |
| --- | --- | --- | --- | --- |
| `productionsPath` | `productions_path` | 1st argument | `""` | `.soar` file sourced at startup. May contain any Soar CLI commands. Empty loads nothing |
| `agentName` | `agent_name` | — | `"robot"` | Soar agent name |
| `decisionsPerInput` | `decisions_per_input` | 2nd argument | `1` | Decision cycles run per input snapshot |
| `debuggerPort` | `debugger_port` | 3rd argument | `0` | TCP port for the Soar debugger. `0` turns it off |
| `maxPendingCommands` | `max_pending_commands` | — | `256` | Commands kept for the robot thread before the oldest are dropped (and marked `^status error`) |
| `realtimePriority` | `realtime_priority` | 4th argument | `0` | SCHED_FIFO priority (1–99) for the Soar thread on Linux. `0` keeps normal scheduling. Use a value below the robot loop's priority. Ignored if the process lacks permission |

=== "C++"

    ```cpp
    soarbridge::SoarHardwareInterface soar{{
        .productionsPath = "/home/lvuser/deploy/robot.soar",
        .decisionsPerInput = 2,
        .realtimePriority = 10,
    }};
    ```

    Designated initializers must follow the declaration order shown in the
    table.

=== "Java"

    ```java
    new SoarBridge(path);                    // defaults
    new SoarBridge(path, 2, 0);              // decisionsPerInput, debuggerPort
    new SoarBridge(path, 2, 0, 10);          // ... realtimePriority
    ```

    `agentName` and `maxPendingCommands` keep their defaults in Java.

=== "Python"

    ```python
    soar = soarbridge.SoarHardwareInterface(
        productions_path="/home/lvuser/py/robot.soar",
        decisions_per_input=2,
        realtime_priority=10,
    )
    ```
