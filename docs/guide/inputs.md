# Inputs by path

The generic input API describes any robot. You choose the names; the bridge
builds the matching structure on the agent's input-link.

## Setting values

=== "Java"

    ```java
    soar.setString("match.mode", "teleop");
    soar.setBool("ground-intake.beam-break", !beamBreak.get());
    soar.setInt("ground-intake.pieces", count);
    soar.setNumber("arm.angle", arm.getAngleDegrees());
    soar.sendInputs();
    ```

=== "C++"

    ```cpp
    m_soar.SetString("match.mode", "teleop");
    m_soar.SetBool("ground-intake.beam-break", !m_beamBreak.Get());
    m_soar.SetInt("ground-intake.pieces", count);
    m_soar.SetNumber("arm.angle", m_arm.GetAngleDegrees());
    m_soar.SendInputs();
    ```

=== "Python"

    ```python
    self.soar.set_string("match.mode", "teleop")
    self.soar.set_bool("ground-intake.beam-break", not self.beam_break.get())
    self.soar.set_int("ground-intake.pieces", count)
    self.soar.set_number("arm.angle", self.arm.get_angle_degrees())
    self.soar.send_inputs()
    ```

The agent then sees:

```soar
(<il> ^match <m> ^ground-intake <g> ^arm <a>)
(<m> ^mode teleop)
(<g> ^beam-break true ^pieces 2)
(<a> ^angle 30.5)
```

and can test it with dotted paths:

```soar
(<il> ^ground-intake.beam-break true ^arm.angle > 10)
```

| Setter | Soar value |
| --- | --- |
| `SetNumber` / `setNumber` / `set_number` | float |
| `SetInt` / `setInt` / `set_int` | int |
| `SetString` / `setString` / `set_string` | string |
| `SetBool` / `setBool` / `set_bool` | the string `true` or `false` |

## Path rules

- **Syntax.** Names separated by dots. Each name starts with a letter and
  contains only letters, digits, `-` and `_`. Do not include a leading `^` or
  `io.input-link`.
- **Parents are created for you.** `SetNumber("a.b.c", 1)` creates `^a` and `^b`
  if they do not exist yet.
- **A path is either a value or a parent, never both.** After
  `SetNumber("arm.angle", 1)`, both `SetNumber("arm.angle.x", 2)` and
  `SetNumber("arm", 2)` are refused. Remove the old path first.
- **Changing type replaces the value.** Setting a string path to a number
  removes the string WME and creates a float one.

A refused call returns `false`, adds one to the `inputErrors` counter, and
explains why in `GetLastWarning()`. Nothing is sent for that path.

## Int or float?

Soar treats `1` (int) and `1.0` (float) as different values. A rule that tests
`^pieces 1` matches `SetInt("...pieces", 1)` but **not**
`SetNumber("...pieces", 1.0)`. Numeric comparisons such as `^angle > 10` work
with both.

Use `SetInt` for counts, IDs and anything you compare with `=`. Use `SetNumber`
for measurements.

## Booleans

Soar has no boolean type. `SetBool` writes the string constant `true` or
`false`, so rules test `^beam-break true`.

## Removing values

`RemoveInput(path)` removes a value, or an identifier and everything under it:

```java
soar.removeInput("vision");  // no target this loop: drop the whole group
```

Removing a value leaves its parent identifier in place, even if it is now
empty. Remove the parent to remove the whole group.

Agents can then test for absence:

```soar
(<il> -^vision)
```

## When values reach the agent

Values are staged until `SendInputs()`. Call it once per loop, after setting
everything. Unchanged values are not sent again, so it is fine (and cheap) to
set every value every loop.

See [How it works](how-it-works.md#inputs-are-snapshots) for snapshot merging,
and [the specification](../io-mapping.md#1-input-link-by-path-robot-soar) for
the exact rules.
