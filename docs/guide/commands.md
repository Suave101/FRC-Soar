# Commands

A command is an **identifier** that the agent puts on its output-link. The
attribute is the command's name and the identifier's children are its
parameters:

```soar
(<ol> ^ground-intake <c>)
(<c> ^state closed ^speed 0.5 ^target <t>)
(<t> ^x 1.5 ^y -2)
```

This arrives in robot code as:

| Field | Value |
| --- | --- |
| `name` | `"ground-intake"` |
| `id` | The command's Soar time tag |
| `params` | `state` → `"closed"`, `speed` → `0.5`, `target.x` → `1.5`, `target.y` → `-2` |

- Nested identifiers become dotted keys, up to 4 levels deep.
- Each value keeps its Soar type: int, float or string.
- `^status` is not a parameter. The bridge writes it.

## Handling commands by name

Register a handler for each command name in robot init, then call
`DispatchCommands()` once per loop:

=== "Java"

    ```java
    // robotInit
    soar.onCommand("ground-intake", c -> groundIntake.setState(c.getString("state")));
    soar.onCommand("arm", c -> arm.setAngle(c.getDouble("angle")));

    // robotPeriodic, after sendInputs()
    soar.dispatchCommands();
    ```

=== "C++"

    ```cpp
    // RobotInit
    m_soar.OnCommand("ground-intake", [this](const soarbridge::Command& c) {
      m_groundIntake.SetState(c.GetString("state"));
    });
    m_soar.OnCommand("arm", [this](const soarbridge::Command& c) {
      m_arm.SetAngle(c.GetNumber("angle"));
    });

    // RobotPeriodic, after SendInputs()
    m_soar.DispatchCommands();
    ```

=== "Python"

    ```python
    # robotInit
    self.soar.on_command("ground-intake", lambda c: self.ground_intake.set_state(c.get_string("state")))
    self.soar.on_command("arm", lambda c: self.arm.set_angle(c.get_number("angle")))

    # robotPeriodic, after send_inputs()
    self.soar.dispatch_commands()
    ```

Handlers run on the thread that calls `DispatchCommands()`, normally the robot
loop, so they can use your subsystems directly. Registering a second handler for
the same name replaces the first.

### Reading parameters

| | C++ (`Command`) | Java (`SoarCommand`) | Python (`Command`) |
| --- | --- | --- | --- |
| Number | `GetNumber(key, fallback = 0.0)` | `getDouble(key)`, `getDouble(key, fallback)` | `get_number(key, default=0.0)` |
| String | `GetString(key, fallback = "")` | `getString(key)`, `getString(key, fallback)` | `get_string(key, default="")` |
| Boolean | `GetBool(key, fallback = false)` | `getBoolean(key, fallback)` | `get_bool(key, default=False)` |
| Present? | `Has(key)` | `has(key)` | `has(key)` |
| All | `params` (`std::map<std::string, Value>`) | `params()` (`Map<String, Object>`) | `params` (`dict`) |

- **Numbers** accept ints, floats, and strings that are entirely a number.
- **Booleans** accept `true` / `false` and `1` / `0`.
- **Strings** return any value as Soar would print it.
- A missing or unconvertible parameter returns the fallback.

In Java, `params()` values are `Long` (Soar int), `Double` (Soar float) or
`String`. In Python they are `int`, `float` or `str`.

## Only registered names are accepted

The bridge rejects any command whose name nobody registered: it gets
`^status error`, is counted in `commandsRejected`, and is explained in
`GetLastWarning()`. A typo in a production therefore shows up as an error
instead of being silently ignored.

The five [built-in commands](fixed-schema.md#built-in-commands) are always
accepted.

## Success and failure

| What happens | `^status` |
| --- | --- |
| Handler returns normally | `complete` |
| Handler throws | `error`. The remaining commands are still dispatched, then the first exception is rethrown |
| Name accepted but no handler at dispatch time | `error`, counted in `commandsUnhandled` |
| Name not registered | `error`, counted in `commandsRejected`, never delivered |
| Parameter given twice (`^v 1 ^v 2`) or nested more than 4 levels | `error`, counted in `commandsRejected`, never delivered |

The full table is in the [specification](../io-mapping.md#5-command-lifecycle).

## Polling by hand (C++ and Python)

Instead of handlers, you can accept a name and handle commands yourself. You
must then report each one back:

=== "C++"

    ```cpp
    m_soar.AcceptCommand("ground-intake");  // RobotInit

    for (const auto& c : m_soar.PollCommands()) {
      if (c.name == "ground-intake") {
        bool ok = m_groundIntake.SetState(c.GetString("state"));
        m_soar.CompleteCommand(c.id, ok);   // ^status complete or error
      } else {
        m_soar.RejectCommand(c.id, "not handled here");
      }
    }
    ```

=== "Python"

    ```python
    self.soar.accept_command("ground-intake")  # robotInit

    for c in self.soar.poll_commands():
        ok = self.ground_intake.set_state(c.get_string("state"))
        self.soar.complete_command(c.id, ok)
    ```

A command you poll but never complete keeps no `^status`, so the agent will
wait on it. Always call `CompleteCommand` or `RejectCommand`.

The Java API dispatches in Java and only offers `onCommand` /
`dispatchCommands`.

## Constants are not commands

```soar
(<ol> ^ground-intake closed)   # wrong: a constant
```

A constant cannot carry `^status`, so the bridge ignores it, counts it in
`outputConstantsIgnored` and explains in `GetLastWarning()`. Write it as an
identifier:

```soar
(<ol> ^ground-intake <c>)
(<c> ^state closed)
```
