# Soar-WPILib Hardware Bridge

Runs a [Soar](https://soar.eecs.umich.edu/) agent inside an FRC robot program.
Robot state is written to the agent's input-link every loop; commands the agent
puts on its output-link are delivered to your subsystems. Usable from C++, Java
and Python (RobotPy).

The WME names are specified in [docs/io-mapping.md](docs/io-mapping.md).

## How it works

```
 robot thread (20 ms loop)                    Soar thread
 -------------------------                    -----------
 SetInputs(snapshot) ──► latest snapshot ──►  update input-link
                                              run 1 decision cycle
 DispatchCommands() ◄─── command queue  ◄───  read output-link
        │
        └─ CompleteCommand ─► completions ──► add ^status complete
```

- The Soar kernel and agent are created, run and destroyed on one background
  thread. Nothing else touches them.
- The robot thread only copies a small struct in or swaps a vector out under a
  mutex that is never held while Soar runs, so it cannot be blocked by the agent.
- If the agent falls behind, newer inputs replace unread ones; the agent always
  sees the latest state and no backlog builds up.
- `^status complete` is added after the robot code has actually executed the
  command, not when the bridge first sees it.

## Layout

| Path | Contents |
| --- | --- |
| `include/soarbridge/` | C++ API: `SoarHardwareInterface.h`, `RobotIO.h`, optional `WpilibAdapter.h` |
| `src/` | C++ core |
| `java/` | SWIG interface and the `frc.soar` Java API |
| `python/` | pybind11 module `soarbridge` |
| `agents/default.soar` | Example agent |
| `cmake/toolchains/` | Cross-compile toolchain files |
| `tests/` | C++, Java and Python end-to-end tests |
| `Soar/` | Soar source checkout (https://github.com/SoarGroup/Soar), built in statically |

## Building

Linux (or WSL) with `g++`, `make` and `python3` + venv. Everything else is
installed under `~/.frc-soar`, not system-wide.

```bash
scripts/setup.sh WPILib_Linux-x64-2027.0.0-alpha-8.tar.gz
```

```bash
scripts/build.sh host
```

`host` builds everything natively and runs the tests. Other targets:

| Target | Output |
| --- | --- |
| `scripts/build.sh host` | x86-64 libraries, jar, Python module; runs tests |
| `scripts/build.sh asan` | Same tests under AddressSanitizer + LeakSanitizer |
| `scripts/build.sh systemcore` | ARM64 for Systemcore (WPILib 2027) and arm64 coprocessors |
| `scripts/build.sh roborio` | ARM32 for roboRIO 1 and roboRIO 2 |

Artifacts land in `~/.frc-soar/build/<target>/dist`:

| File | Use |
| --- | --- |
| `libsoarbridge.so` | C++ robot projects |
| `libsoarbridge_jni.so` + `soarbridge.jar` | Java robot projects (the jar also embeds the `.so`) |
| `python/soarbridge.*.so` | RobotPy projects |

Each `.so` contains Soar and SQLite with their symbols hidden; there is nothing
else to deploy.

### Targets and toolchains

- **ARM32 / roboRIO.** The roboRIO 1 and roboRIO 2 are both 32-bit ARMv7
  (Cortex-A9, soft-float ABI) and take the same binary. It must be built with
  the WPILib roboRIO toolchain (`arm-frc<year>-linux-gnueabi-g++`); a generic
  `arm-linux-gnueabihf` compiler produces incompatible code. The toolchain is
  not in the WPILib 2027 installer: `scripts/setup.sh --roborio` downloads it
  (opensdk v2025-2, GCC 12.1, ~125 MB) into `~/.frc-soar/roborio`, or set
  `ROBORIO_TOOLCHAIN` to the root of an existing WPILib 2025/2026 toolchain.
  The jar packs the library under `native/linux-arm/`, matching the roboRIO
  JVM's `os.arch`.
- **ARM64 / Systemcore.** Uses the `aarch64-systemcore2027-linux-gnu` toolchain
  from the WPILib 2027 installer (unpacked by `scripts/setup.sh`), or a distro
  `aarch64-linux-gnu-g++`. Override with `SYSTEMCORE_TOOLCHAIN`.
- **Python when cross compiling.** Needs the target's Python headers:
  `-DSOARBRIDGE_PYTHON_INCLUDE_DIR=... -DSOARBRIDGE_PYTHON_EXT_SUFFIX=...`.
  Without them the Python module is skipped for that target.

## Usage

### C++

```cpp
#include <soarbridge/SoarHardwareInterface.h>
#include <soarbridge/WpilibAdapter.h>   // optional helpers

class Robot : public frc::TimedRobot, soarbridge::CommandHandler {
  soarbridge::SoarHardwareInterface m_soar{
      {.productionsPath = frc::filesystem::GetDeployDirectory() + "/robot.soar"}};
  soarbridge::RobotInputs m_in;

  void RobotInit() override {
    if (!m_soar.Start()) FRC_ReportError(frc::err::Error, "{}", m_soar.GetLastError());
  }

  void RobotPeriodic() override {
    soarbridge::wpilib::ReadMatchState(m_in);
    soarbridge::wpilib::ReadDrive(m_in, m_drive.GetPose(), m_drive.GetFieldVelocity());
    m_in.targetDetected = m_vision.HasTarget();
    m_in.hasPiece = m_intake.HasPiece();
    m_in.elevatorHeight = m_elevator.GetHeight().value();
    m_soar.SetInputs(m_in);

    // ... rest of the loop ...

    m_soar.DispatchCommands(*this);
  }

  void Move(double vx, double vy, double omega) override { m_drive.Drive(vx, vy, omega); }
  void TurnTo(double heading) override { m_drive.TurnTo(heading); }
  void SetElevator(double height) override { m_elevator.SetHeight(height); }
  void RunIntake(double speed) override { m_intake.Set(speed); }
  void Shoot(bool execute) override { if (execute) m_shooter.Fire(); }
};
```

`WpilibAdapter.h` supports both the 2027 `wpi::` API and the 2020-2026 `frc::`
API. The library itself does not link WPILib.

### Java

```java
import frc.soar.*;

public class Robot extends TimedRobot implements RobotActuators {
  private SoarBridge soar;

  @Override public void robotInit() {
    soar = new SoarBridge(Filesystem.getDeployDirectory() + "/robot.soar");
    soar.start();
  }

  @Override public void robotPeriodic() {
    soar.setMatch(DriverStation.getMatchTime(), mode(), alliance());
    soar.setDrive(pose.getX(), pose.getY(), pose.getRotation().getDegrees(), vx, vy);
    soar.setVision(vision.hasTarget(), vision.distance(), vision.yaw());
    soar.setIntakeHasPiece(intake.hasPiece());
    soar.setElevatorHeight(elevator.getHeight());
    soar.sendInputs();

    // ... rest of the loop ...

    soar.dispatchCommands(this);
  }

  @Override public void move(double vx, double vy, double omega) { drive.drive(vx, vy, omega); }
  // turnTo, setElevator, runIntake, shoot ...
}
```

Add `soarbridge.jar` to the classpath. The native library is loaded
automatically, from `java.library.path` if present, otherwise from inside the
jar. `SoarBridge` is `AutoCloseable` and owns all native memory.

### Python

```python
import soarbridge

class Robot(wpilib.TimedRobot, soarbridge.CommandHandler):
    def robotInit(self):
        soarbridge.CommandHandler.__init__(self)
        self.soar = soarbridge.SoarHardwareInterface(productions_path="/home/lvuser/py/robot.soar")
        self.soar.start()
        self.inputs = soarbridge.RobotInputs()

    def robotPeriodic(self):
        self.inputs.match_time = wpilib.DriverStation.getMatchTime()
        self.inputs.mode = soarbridge.MatchMode.TELEOP
        self.inputs.has_piece = self.intake.has_piece()
        self.soar.set_inputs(self.inputs)
        # ... rest of the loop ...
        self.soar.dispatch_commands(self)

    def move(self, vx, vy, omega): self.drive.drive(vx, vy, omega)
    def turn_to(self, heading): ...
    def set_elevator(self, height): ...
    def run_intake(self, speed): ...
    def shoot(self, execute): ...
```

## Configuration

| Option (C++ / Python) | Default | Meaning |
| --- | --- | --- |
| `productionsPath` / `productions_path` | — | `.soar` file sourced at startup |
| `agentName` / `agent_name` | `robot` | Soar agent name |
| `decisionsPerInput` / `decisions_per_input` | 1 | Decision cycles per input snapshot |
| `debuggerPort` / `debugger_port` | 0 | TCP port for attaching the Soar debugger; 0 = off |
| `maxPendingCommands` / `max_pending_commands` | 256 | Undelivered commands kept before dropping the oldest |
| `realtimePriority` / `realtime_priority` | 0 | SCHED_FIFO priority for the Soar thread; 0 = normal scheduling |

`GetStats()` reports counters and input-to-command latency (last, mean, max,
and how many inputs exceeded 5 ms); publish these to the dashboard to watch the
agent on the real robot.

## Writing agents

- One decision cycle runs per robot loop by default, so an operator proposed
  from this loop's inputs has its commands delivered in this same loop.
- Remove commands once they carry `^status`; see `agents/default.soar`.
- Give operators that can be proposed several at once an indifferent (`=`)
  preference. A tie impasse with no rules to resolve it stalls the agent.
- The bridge forces `soar stop-phase input` and `trace 0`. Other settings can
  go in the productions file.
