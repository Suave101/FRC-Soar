# How it works

```
 robot thread (20 ms loop)                    Soar thread
 -------------------------                    -----------
 Set*(path, value)   (staged locally)
 SendInputs()        ──► changed values  ──►  update input-link
                                              run decision cycle(s)
 DispatchCommands() ◄─── command queue  ◄───  read output-link
        │
        └─ CompleteCommand ─► completions ──► add ^status complete
```

## Two threads

`Start()` creates a background thread. That thread creates the Soar kernel and
agent, loads your productions, runs the agent, and destroys it all on
`Stop()`. No other thread ever touches Soar.

The robot thread talks to the Soar thread through a small shared area protected
by one mutex. The Soar thread only holds that mutex to copy data in or out,
**never while the agent is running**. So every call you make from the robot
loop returns quickly, however long the agent takes to think.

## Inputs are snapshots

The `Set*` methods only record values on the robot side. Nothing reaches Soar
until you call `SendInputs()`, which hands over everything that changed since
the last call as one snapshot. The agent therefore always sees one loop's
values together, never half of one loop and half of the next.

- **Unchanged values are not sent again.** A beam-break that reads `false` for
  a hundred loops is written to working memory once, so it does not "blink"
  (get removed and re-added) and re-fire rules.
- **If the agent falls behind, snapshots merge.** When Soar has not yet used
  the previous snapshot, the new one is merged into it and the newest value of
  each path wins. The agent always works on the latest state and no backlog
  builds up. `BridgeStats::inputsOverwritten` counts how often this happens.

## The agent runs once per snapshot

Each snapshot wakes the Soar thread. It applies the changes to the input-link,
runs `decisionsPerInput` decision cycles (default 1), and then reads the
output-link.

!!! note
    The agent only runs when you call `SendInputs()` (or `SetInputs()`). If your
    robot code stops sending inputs, the agent stops thinking.

The bridge sets Soar's stop phase to `input`. This means one decision cycle
covers the whole input → propose → decide → apply → output sequence. An
operator chosen from this loop's inputs puts its commands on the output-link in
the same run, not one loop later.

## Commands are delivered once

After each run, the bridge looks at what is new on the output-link:

- Each new **identifier** is a command. If its name is registered (or
  built in) and its parameters are valid, it is queued for the robot thread.
  Otherwise it gets `^status error` at once.
- Each new **constant** (such as `^grip closed`) cannot carry a `^status`, so
  it is ignored, counted and reported as a warning.

`DispatchCommands()` (or `PollCommands()`) takes everything in the queue. A
command is delivered to the robot exactly once, however long it stays on the
output-link.

## Completion is real

When your handler returns, the bridge records a completion. The Soar thread
adds `^status complete` to the command (or `^status error` if the handler
threw). The agent sees it on its next decision cycle and should remove the
command. Until then, rules can test for the command's presence to avoid
issuing it twice.

```
loop N    SendInputs → agent proposes and applies "grab" → ^ground-intake <c>
          DispatchCommands → your handler runs → completion recorded
          Soar thread adds (<c> ^status complete)
loop N+1  SendInputs → agent sees ^status, removes the command
```

## Timing

With the default settings, inputs sent at the start of the loop usually produce
their commands well before `DispatchCommands()` is called later in the same
loop. If the agent has not finished yet, its commands are simply delivered at
the next `DispatchCommands()`.

`GetStats()` measures the time from `SendInputs()` to the resulting commands
being ready to dispatch: the last, mean and maximum, plus how many inputs took
longer than 5 ms. See [Telemetry and debugging](telemetry.md).

## Back-pressure

If the robot never dispatches commands, the queue is capped at
`maxPendingCommands` (default 256). Beyond that the oldest commands are dropped
and marked `^status error`, so the agent can clear them and try again instead
of waiting for ever.

## Restarting

`Stop()` destroys the agent. A later `Start()` creates a fresh one and reloads
the productions. Every input path you have set is sent again at the next
`SendInputs()`, and registered command names are kept, so robot code does not
need to set anything up again.
