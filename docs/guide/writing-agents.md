# Writing agents

This page covers what is specific to running a Soar agent through the bridge.
For Soar itself, see the [Soar manual and tutorials](https://soar.eecs.umich.edu/).

## The productions file

The file you pass as `productionsPath` is sourced when the bridge starts. It
can contain productions and any Soar command-line settings. If it fails to
load, `Start()` fails and `GetLastError()` says why.

After loading the file, the bridge always sets:

- `soar stop-phase input`, so each run covers a full decision cycle from input
  to output. See [How it works](how-it-works.md#the-agent-runs-once-per-snapshot).
- `trace 0`, because tracing costs time on the robot and nobody reads it.

## Rules every agent needs

### Remove finished commands

The bridge adds `^status` to each command once the robot has handled it (or
refused it). The agent must remove it. Otherwise rules that test for "no
command yet" never fire again.

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

The `>` preference makes clearing win over other operators. The `=` preference
lets several commands finish in the same cycle without a tie.

### Wait when there is nothing to do

When no operator is proposed, Soar creates a state-no-change impasse each
cycle. A `wait` operator in the substate keeps that from stacking up:

```soar
sp {robot*propose*wait
   (state <s> ^attribute state ^choices none -^operator.name wait)
-->
   (<s> ^operator <o> +)
   (<o> ^name wait)}
```

## Guidelines

**Commands must be identifiers.** `(<ol> ^grip <c>) (<c> ^state closed)`, not
`(<ol> ^grip closed)`. A constant cannot get a `^status` and is ignored.

**Do not issue a command while the same one is pending.** Test that the command
is absent before proposing it:

```soar
sp {robot*propose*shoot
   (state <s> ^superstate nil ^io <io>)
   (<io> ^input-link.intake.has-piece true ^output-link <ol>)
  -(<ol> ^shoot)
-->
   (<s> ^operator <o> + =)
   (<o> ^name shoot)}
```

**Give operators that can be proposed together an indifferent (`=`)
preference.** A tie impasse with no rules to resolve it stalls the agent.

**Match types exactly.** Booleans are the strings `true` / `false`. Ints and
floats are different: `^count 1` does not match a float `1.0`. See
[Int or float?](inputs.md#int-or-float).

**Expect to handle errors.** A command can come back with `^status error` if
its name is not registered, a parameter is missing, the handler threw, or the
queue overflowed. The clear rule above removes it either way. Test
`^status error` if the agent should react.

**Think in loops.** By default the agent runs one decision cycle per robot loop
(every 20 ms). Use `decisionsPerInput` to run more cycles per loop if the agent
needs several steps to decide. Check the latency in
[Telemetry and debugging](telemetry.md) when you do.

## Examples

- [`agents/default.soar`](https://github.com/Suave101/FRC-Soar/blob/main/agents/default.soar):
  the fixed schema. Chases a vision target and shoots.
- [`tests/generic_io.soar`](https://github.com/Suave101/FRC-Soar/blob/main/tests/generic_io.soar):
  the generic layer, including commands the bridge must refuse.
- [Your first agent](../getting-started/first-agent.md): a small agent with its
  own subsystem.
