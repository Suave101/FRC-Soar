# Soar-WPILib Hardware Bridge

Run a [Soar](https://soar.eecs.umich.edu/) cognitive agent inside an FRC robot
program. Each loop, the bridge writes the robot's state to the agent's
**input-link**. Commands the agent puts on its **output-link** are delivered to
your subsystems. It works from **C++**, **Java** and **Python (RobotPy)**, on the
**roboRIO 1/2** (ARM32) and **Systemcore** (ARM64).

```java
// robotInit
soar.onCommand("ground-intake", c -> groundIntake.setState(c.getString("state")));

// robotPeriodic
soar.setBool("ground-intake.beam-break", !beamBreak.get());
soar.setNumber("arm.angle", arm.getAngleDegrees());
soar.sendInputs();
// ... rest of the loop ...
soar.dispatchCommands();
```

```soar
sp {robot*apply*grab
   (state <s> ^operator.name grab ^io.output-link <ol>)
-->
   (<ol> ^ground-intake <c>)
   (<c> ^state closed)}
```

## Why use it

- **The robot loop never waits on the agent.** Soar runs on its own thread.
  Calls from the 20 ms loop only copy a few values under a lock that is never
  held while Soar runs.
- **The agent always sees fresh data.** If the agent falls behind, newer inputs
  replace unread ones, so no backlog builds up.
- **Completion means the robot actually executed the command.** `^status complete`
  is written after your handler runs, not when the bridge first reads the
  command.
- **Mistakes in productions are reported, not silently ignored.** Unknown command
  names, missing parameters and malformed commands get `^status error`, a counter
  and a readable warning.
- **Nothing extra to deploy.** Each library is a single `.so` with Soar and
  SQLite built in and their symbols hidden.

## Two ways to describe your robot

| | Generic layer | Fixed schema |
| --- | --- | --- |
| Inputs | `SetNumber` / `SetInt` / `SetString` / `SetBool` by dotted path | A `RobotInputs` struct (match, drive, vision, intake, elevator) |
| Commands | Any name you register with `OnCommand` | `move`, `turn-to`, `set-elevator`, `run-intake`, `shoot` |
| Use when | Your robot has its own subsystems | The built-in schema fits your robot |

The fixed schema is built on the generic layer, and both can be used together.
Start with [Inputs by path](guide/inputs.md) and [Commands](guide/commands.md),
or with [The fixed schema](guide/fixed-schema.md).

## Where to go next

- [Building](getting-started/building.md): build the libraries and run the tests.
- [Your first agent](getting-started/first-agent.md): a complete robot program
  and agent, in all three languages.
- [How it works](guide/how-it-works.md): the threading model, snapshots and the
  command lifecycle.
- [I/O mapping specification](io-mapping.md): the exact working-memory elements
  and rules.
