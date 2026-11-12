# Troubleshooting

Start with `GetLastWarning()` (Java `lastWarning()`, Python
`get_last_warning()`). It usually says exactly what went wrong.

## `Start()` fails

`GetLastError()` (Java: the exception message) explains why.

- **"Failed to load productions from ..."**: the file is missing, or a
  production has a syntax error. Check the path: deploy-directory paths are
  different on the robot and in simulation. Load the file in the Soar CLI or
  debugger to find the error.
- **Soar kernel or agent creation failed**: if `debuggerPort` is set, the port
  may already be in use. Try another port, or `0`.

## The agent never does anything

- Is `SendInputs()` called every loop? **The agent only runs when inputs are
  sent.** Check that `inputsProcessed` increases.
- Do your rules test the values you actually set? Booleans are the strings
  `true` / `false`, and `1` (int) does not match `1.0` (float). See
  [Int or float?](guide/inputs.md#int-or-float).
- Is `DispatchCommands()` called every loop? Commands wait until it is.
- Do your proposals have `=` preferences? Two operators proposed together
  without one cause a tie impasse that stalls the agent.

## A command is never delivered

| Symptom | Cause | Fix |
| --- | --- | --- |
| `commandsRejected` increases, warning says "unknown command" | Name not registered | Call `OnCommand` (or `AcceptCommand`) for that name in robot init, or fix the typo in the production |
| `commandsRejected` increases for `move`, `shoot`, ... | A required parameter is missing or not a number | See [built-in commands](guide/fixed-schema.md#built-in-commands) |
| `commandsRejected` increases, warning mentions a parameter | A parameter given twice, or nested more than 4 levels | Give each parameter once |
| `outputConstantsIgnored` increases | `(<ol> ^name value)` is a constant, not a command | Use `(<ol> ^name <c>) (<c> ^value value)` |
| `commandsUnhandled` increases | Name accepted, but no handler at dispatch time | Register a handler, or pass a `CommandHandler` for built-ins |
| `commandsDropped` increases | `DispatchCommands()` is not called often enough | Call it every loop |

## A command is only issued once

The agent must remove finished commands. Add the
[clear-commands rules](guide/writing-agents.md#remove-finished-commands).
Without them, a rule that tests `-(<ol> ^shoot)` never fires again.

## Commands arrive a loop late

Check `GetStats()`. If the latency is often close to or over the loop period,
the agent is not finishing before `DispatchCommands()` runs:

- call `SendInputs()` as early in the loop as possible, and
  `DispatchCommands()` as late as possible;
- set `realtimePriority` so other threads do not delay the Soar thread;
- lower `decisionsPerInput` or simplify the productions.

## A setter returns `false`

The path is invalid or conflicts with an existing one. Common causes:

- a name that starts with a digit or contains `.` / spaces in the wrong place;
- setting `"arm"` after `"arm.angle"` (a path is either a value or a parent);
- setting `"match"` or another fixed-schema path as a value after
  `SetInputs()` has been used.

Use `RemoveInput` first to change a path's shape. See
[Path rules](guide/inputs.md#path-rules).

## Java: `UnsatisfiedLinkError`

The jar does not contain a native library for this platform. A jar built with
`roborio` contains only `linux-arm`, one built with `systemcore` contains only
`linux-aarch64`, and one built with `host` contains only `linux-amd64`. Build
for the right target, or put the matching `libsoarbridge_jni.so` on
`java.library.path`. Windows and macOS are not supported.

## Python: `ImportError`

The module must match both the robot's CPU and its Python version. Rebuild with
the target's headers and extension suffix. See
[Cross-compiling notes](getting-started/building.md#cross-compiling-notes).
