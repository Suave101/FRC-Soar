# Your first agent

This page builds a small robot with one subsystem, a ground intake that has a
beam-break sensor and two states (`open` and `closed`). The agent closes the
intake when a game piece breaks the beam during teleop, and opens it again when
the beam clears.

It uses the [generic layer](../guide/inputs.md), which works for any robot. To
use the built-in drive/vision/intake schema instead, see
[The fixed schema](../guide/fixed-schema.md).

## The agent

Save this as `robot.soar` in your project's deploy directory
(`src/main/deploy/` for C++ and Java projects).

```soar
# Remove commands once the bridge has marked them ^status complete or error.
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

# A piece is in the intake and we have not closed it yet: close it.
sp {robot*propose*grab
   (state <s> ^superstate nil ^io <io>)
   (<io> ^input-link <il> ^output-link <ol>)
   (<il> ^match.mode teleop
         ^ground-intake.beam-break true)
  -(<s> ^intake-state closed)
  -(<ol> ^ground-intake)
-->
   (<s> ^operator <o> + =)
   (<o> ^name grab)}

sp {robot*apply*grab
   (state <s> ^operator.name grab ^io.output-link <ol>)
-->
   (<s> ^intake-state closed)
   (<ol> ^ground-intake <c>)
   (<c> ^state closed)}

# The beam is clear and the intake is closed: open it.
sp {robot*propose*release
   (state <s> ^superstate nil ^intake-state closed ^io <io>)
   (<io> ^input-link.ground-intake.beam-break false ^output-link <ol>)
  -(<ol> ^ground-intake)
-->
   (<s> ^operator <o> + =)
   (<o> ^name release)}

sp {robot*apply*release
   (state <s> ^operator.name release ^io.output-link <ol>)
-->
   (<s> ^intake-state closed -)
   (<ol> ^ground-intake <c>)
   (<c> ^state open)}

# Nothing to do: wait quietly instead of stacking impasses.
sp {robot*propose*wait
   (state <s> ^attribute state ^choices none -^operator.name wait)
-->
   (<s> ^operator <o> +)
   (<o> ^name wait)}
```

Every agent needs the two `clear-commands` rules and the `wait` rule. See
[Writing agents](../guide/writing-agents.md) for why.

## The robot program

=== "Java"

    ```java
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj.Filesystem;
    import edu.wpi.first.wpilibj.TimedRobot;
    import frc.soar.SoarBridge;

    public class Robot extends TimedRobot {
      private final GroundIntake groundIntake = new GroundIntake();
      private SoarBridge soar;

      @Override
      public void robotInit() {
        soar = new SoarBridge(Filesystem.getDeployDirectory() + "/robot.soar");
        soar.start();  // throws IllegalStateException if robot.soar fails to load
        soar.onCommand("ground-intake", c -> groundIntake.setState(c.getString("state")));
      }

      @Override
      public void robotPeriodic() {
        soar.setString("match.mode", DriverStation.isTeleop() ? "teleop" : "disabled");
        soar.setBool("ground-intake.beam-break", groundIntake.beamBroken());
        soar.sendInputs();

        // ... the rest of your loop ...

        soar.dispatchCommands();
      }
    }
    ```

=== "C++"

    ```cpp
    #include <frc/DriverStation.h>
    #include <frc/Errors.h>
    #include <frc/Filesystem.h>
    #include <frc/TimedRobot.h>
    #include <soarbridge/SoarHardwareInterface.h>

    class Robot : public frc::TimedRobot {
      GroundIntake m_groundIntake;
      soarbridge::SoarHardwareInterface m_soar{
          {.productionsPath = frc::filesystem::GetDeployDirectory() + "/robot.soar"}};

     public:
      void RobotInit() override {
        if (!m_soar.Start()) {
          FRC_ReportError(frc::err::Error, "{}", m_soar.GetLastError());
        }
        m_soar.OnCommand("ground-intake", [this](const soarbridge::Command& c) {
          m_groundIntake.SetState(c.GetString("state"));
        });
      }

      void RobotPeriodic() override {
        m_soar.SetString("match.mode", frc::DriverStation::IsTeleop() ? "teleop" : "disabled");
        m_soar.SetBool("ground-intake.beam-break", m_groundIntake.BeamBroken());
        m_soar.SendInputs();

        // ... the rest of your loop ...

        m_soar.DispatchCommands();
      }
    };
    ```

=== "Python"

    ```python
    import wpilib
    import soarbridge


    class Robot(wpilib.TimedRobot):
        def robotInit(self):
            self.ground_intake = GroundIntake()
            self.soar = soarbridge.SoarHardwareInterface(
                productions_path="/home/lvuser/py/robot.soar")
            self.soar.start()  # raises RuntimeError if robot.soar fails to load
            self.soar.on_command(
                "ground-intake",
                lambda c: self.ground_intake.set_state(c.get_string("state")))

        def robotPeriodic(self):
            teleop = wpilib.DriverStation.isTeleop()
            self.soar.set_string("match.mode", "teleop" if teleop else "disabled")
            self.soar.set_bool("ground-intake.beam-break", self.ground_intake.beam_broken())
            self.soar.send_inputs()

            # ... the rest of your loop ...

            self.soar.dispatch_commands()
    ```

## What happens each loop

1. The setters stage `^match.mode` and `^ground-intake.beam-break`. A value that
   has not changed since the last loop is not sent again.
2. `sendInputs()` publishes them together as one snapshot. The Soar thread
   updates the input-link and runs one decision cycle.
3. If the agent chose `grab`, a `^ground-intake` command with `^state closed`
   appears on the output-link, and the bridge queues it for the robot thread.
4. `dispatchCommands()` calls your `ground-intake` handler, then tells the
   bridge the command ran. The bridge adds `^status complete` to it.
5. On its next decision cycle the agent sees `^status` and removes the command.

With the default settings, the agent sees this loop's inputs in time for its
commands to be dispatched later in the same loop. See
[How it works](../guide/how-it-works.md) for details.

## Check that it is working

Publish a few counters to the dashboard while you develop:

```java
SmartDashboard.putNumber("Soar/inputs", soar.inputsProcessed());
SmartDashboard.putNumber("Soar/latency ms", soar.meanLatencyMillis());
SmartDashboard.putNumber("Soar/rejected", soar.commandsRejected());
SmartDashboard.putString("Soar/warning", soar.lastWarning());
```

If `rejected` goes up or a warning appears, see
[Troubleshooting](../troubleshooting.md).
