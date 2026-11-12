# The fixed schema

For a typical robot with a drivetrain, vision, an intake, an elevator and a
shooter, the bridge has a ready-made schema: a `RobotInputs` struct for inputs
and five built-in commands. It is built on the [generic layer](inputs.md), and
you can add your own paths and commands next to it.

## Inputs

Fill a `RobotInputs` and pass it to `SetInputs()`. It writes each field to its
path, then calls `SendInputs()`.

| WME | Type | C++ field | Python field | Meaning |
| --- | --- | --- | --- | --- |
| `^match.time` | float | `matchTime` | `match_time` | Match time remaining, seconds |
| `^match.mode` | string | `mode` | `mode` | `disabled`, `autonomous`, `teleop` or `test` |
| `^match.alliance` | string | `alliance` | `alliance` | `red` or `blue` |
| `^drive.pose.x` | float | `poseX` | `pose_x` | X position, meters |
| `^drive.pose.y` | float | `poseY` | `pose_y` | Y position, meters |
| `^drive.pose.heading` | float | `heading` | `heading` | Gyro heading, degrees |
| `^drive.velocity.vx` | float | `velocityX` | `velocity_x` | X velocity, m/s |
| `^drive.velocity.vy` | float | `velocityY` | `velocity_y` | Y velocity, m/s |
| `^vision.target-detected` | boolean | `targetDetected` | `target_detected` | Target in view |
| `^vision.target-distance` | float | `targetDistance` | `target_distance` | Distance to target, meters |
| `^vision.target-yaw` | float | `targetYaw` | `target_yaw` | Angle offset to target, degrees |
| `^intake.has-piece` | boolean | `hasPiece` | `has_piece` | Intake holds a game piece |
| `^elevator.height` | float | `elevatorHeight` | `elevator_height` | Elevator extension, meters |

These paths only appear once you call `SetInputs()`. Robots that use only the
generic setters never see them.

You can add your own paths alongside, such as `SetNumber("intake.current", 12.5)`.
You cannot turn one of the schema's paths into a parent or a value: once
`SetInputs()` has been used, `SetNumber("match", 1)` is refused.

=== "Java"

    Java has grouped setters instead of a struct. Once you use any of them,
    `sendInputs()` writes the whole schema.

    ```java
    soar.setMatch(DriverStation.getMatchTime(), MatchMode.Teleop, Alliance.Blue);
    soar.setDrive(pose.getX(), pose.getY(), pose.getRotation().getDegrees(), vx, vy);
    soar.setVision(vision.hasTarget(), vision.distance(), vision.yaw());
    soar.setIntakeHasPiece(intake.hasPiece());
    soar.setElevatorHeight(elevator.getHeight());
    soar.sendInputs();
    ```

=== "C++"

    ```cpp
    soarbridge::RobotInputs in;
    soarbridge::wpilib::ReadMatchState(in);   // optional WPILib helpers
    soarbridge::wpilib::ReadDrive(in, m_drive.GetPose(), m_drive.GetFieldVelocity());
    in.targetDetected = m_vision.HasTarget();
    in.hasPiece = m_intake.HasPiece();
    in.elevatorHeight = m_elevator.GetHeight().value();
    m_soar.SetInputs(in);
    ```

=== "Python"

    ```python
    inputs = soarbridge.RobotInputs()
    inputs.match_time = wpilib.DriverStation.getMatchTime()
    inputs.mode = soarbridge.MatchMode.TELEOP
    inputs.alliance = soarbridge.Alliance.BLUE
    inputs.has_piece = self.intake.has_piece()
    self.soar.set_inputs(inputs)
    ```

### WPILib helpers (C++)

`soarbridge/WpilibAdapter.h` is an optional, header-only helper that fills
`RobotInputs` from WPILib. The library itself does not link WPILib, so include
this header only in robot code.

| Function | Fills |
| --- | --- |
| `soarbridge::wpilib::ReadMatchState(in)` | `matchTime`, `mode`, `alliance` from the Driver Station. The alliance is left unchanged until the Driver Station reports one |
| `soarbridge::wpilib::ReadDrive(in, pose, velocity)` | `poseX`, `poseY`, `heading`, `velocityX`, `velocityY` from a `Pose2d` and chassis velocity |

It supports both WPILib API generations:

- **2027 and later (Systemcore):** the `wpi::` namespace and `.hpp` headers.
  "Utility" mode maps to `test`.
- **2020–2026 (roboRIO):** the `frc::` namespace and `.h` headers.

Pass field-relative velocities to `ReadDrive` if your agent reasons in field
coordinates.

## Built-in commands

These five commands are always accepted. Every listed parameter is required.
If one is missing or has the wrong type, the bridge rejects the command with
`^status error`.

| Command | Parameters | C++ handler | Java (`RobotActuators`) | Python (`CommandHandler`) |
| --- | --- | --- | --- | --- |
| `^move` | `^vx`, `^vy` (m/s), `^omega` (rad/s) | `Move(vx, vy, omega)` | `move(vx, vy, omega)` | `move(vx, vy, omega)` |
| `^turn-to` | `^heading` (degrees, absolute) | `TurnTo(heading)` | `turnTo(heading)` | `turn_to(heading)` |
| `^set-elevator` | `^height` (meters) | `SetElevator(height)` | `setElevator(height)` | `set_elevator(height)` |
| `^run-intake` | `^speed` (-1.0 to 1.0) | `RunIntake(speed)` | `runIntake(speed)` | `run_intake(speed)` |
| `^shoot` | `^execute` (boolean) | `Shoot(execute)` | `shoot(execute)` | `shoot(execute)` |

Numeric parameters may be written as integers (`^speed 1`). `^execute` also
accepts `1` / `0`.

Implement the handler interface and pass it to `DispatchCommands`:

=== "Java"

    ```java
    public class Robot extends TimedRobot implements RobotActuators {
      @Override public void robotPeriodic() {
        // ... inputs ...
        soar.dispatchCommands(this);
      }

      @Override public void move(double vx, double vy, double omega) { drive.drive(vx, vy, omega); }
      @Override public void turnTo(double heading) { drive.turnTo(heading); }
      @Override public void setElevator(double height) { elevator.setHeight(height); }
      @Override public void runIntake(double speed) { intake.set(speed); }
      @Override public void shoot(boolean execute) { if (execute) shooter.fire(); }
    }
    ```

=== "C++"

    ```cpp
    class Robot : public frc::TimedRobot, soarbridge::CommandHandler {
      void RobotPeriodic() override {
        // ... inputs ...
        m_soar.DispatchCommands(*this);
      }

      void Move(double vx, double vy, double omega) override { m_drive.Drive(vx, vy, omega); }
      void TurnTo(double heading) override { m_drive.TurnTo(heading); }
      void SetElevator(double height) override { m_elevator.SetHeight(height); }
      void RunIntake(double speed) override { m_intake.Set(speed); }
      void Shoot(bool execute) override { if (execute) m_shooter.Fire(); }
    };
    ```

=== "Python"

    ```python
    class Robot(wpilib.TimedRobot, soarbridge.CommandHandler):
        def robotInit(self):
            soarbridge.CommandHandler.__init__(self)  # required
            ...

        def robotPeriodic(self):
            # ... inputs ...
            self.soar.dispatch_commands(self)

        def move(self, vx, vy, omega): self.drive.drive(vx, vy, omega)
        def turn_to(self, heading): self.drive.turn_to(heading)
        def set_elevator(self, height): self.elevator.set_height(height)
        def run_intake(self, speed): self.intake.set(speed)
        def shoot(self, execute):
            if execute:
                self.shooter.fire()
    ```

    Python subclasses must call `soarbridge.CommandHandler.__init__(self)`.

A handler registered with `OnCommand` for a built-in name takes precedence over
the handler interface. Custom commands registered with `OnCommand` are
dispatched by the same `DispatchCommands(handler)` call.

## Example agent

[`agents/default.soar`](https://github.com/Suave101/FRC-Soar/blob/main/agents/default.soar)
uses the fixed schema. While enabled, it drives at the vision target with the
intake running until it holds a piece, then shoots.
