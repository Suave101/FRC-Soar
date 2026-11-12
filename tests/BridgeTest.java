import frc.soar.Alliance;
import frc.soar.MatchMode;
import frc.soar.RobotActuators;
import frc.soar.SoarBridge;
import frc.soar.SoarCommand;
import java.util.ArrayList;
import java.util.List;

/** Exercises the Java API end to end. Usage: BridgeTest <agents dir> */
public final class BridgeTest {
  private static int failures;

  private static void check(boolean condition, String what) {
    if (!condition) {
      failures++;
      System.out.println("  FAIL: " + what);
    }
  }

  private static final class Recorder implements RobotActuators {
    int moves, intakes, shots;
    double omega;

    @Override public void move(double vx, double vy, double omega) { moves++; this.omega = omega; }
    @Override public void turnTo(double headingDegrees) {}
    @Override public void setElevator(double heightMeters) {}
    @Override public void runIntake(double speed) { intakes++; }
    @Override public void shoot(boolean execute) { if (execute) shots++; }
  }

  /** Path inputs and named commands against tests/generic_io.soar. */
  private static void generic(String tests) throws Exception {
    List<SoarCommand> reports = new ArrayList<>();
    List<SoarCommand> beams = new ArrayList<>();
    try (SoarBridge soar = new SoarBridge(tests + "/generic_io.soar")) {
      soar.start();
      soar.onCommand("report", reports::add);
      soar.onCommand("beam", beams::add);
      soar.onCommand("angle", c -> { });
      soar.onCommand("has-piece", c -> { });
      soar.onCommand("no-vision", c -> { });

      check(soar.setString("match.mode", "teleop"), "setString");
      check(soar.setInt("ground-intake.beam-break", 1), "setInt");
      check(soar.setBool("ground-intake.has-piece", true), "setBool");
      check(soar.setNumber("arm.angle", 30.5), "setNumber");
      check(!soar.setNumber("ground-intake", 1.0), "parent path refused as a value");
      check(soar.setInt("test.seq", 1), "set seq");
      for (int i = 0; i < 50 && reports.isEmpty(); i++) {
        soar.sendInputs();  // no typed setter used: only these paths are written
        Thread.sleep(2);
        soar.dispatchCommands();
      }
      check(reports.size() == 1, "one report, got " + reports.size());
      if (!reports.isEmpty()) {
        SoarCommand r = reports.get(0);
        check(r.name().equals("report"), "name");
        check(Long.valueOf(1).equals(r.params().get("seq")), "Soar int -> Long");
        check(r.getString("mode").equals("teleop"), "mode param");
        check(Math.abs(r.getDouble("target.x") - 1.5) < 1e-9, "nested target.x");
        check(Long.valueOf(-2).equals(r.params().get("target.y")), "nested target.y");
        check(r.params().size() == 4, "4 params, got " + r.params());
      }
      check(beams.size() == 1 && Long.valueOf(1).equals(beams.get(0).params().get("value")),
          "beam carries the int input");
      check(soar.commandsRejected() == 2, "mystery + dup rejected, got " + soar.commandsRejected());
      check(soar.outputConstantsIgnored() == 1, "^legacy constant counted");
      check(soar.inputErrors() == 1, "one refused path");
      check(!soar.lastWarning().isEmpty(), "warning reported");

      boolean threw = false;
      try {
        soar.onCommand("bad name", c -> { });
      } catch (IllegalArgumentException expected) {
        threw = true;
      }
      check(threw, "invalid command name throws");
    }
  }

  public static void main(String[] args) throws Exception {
    String agents = args[0];

    boolean threw = false;
    try (SoarBridge bad = new SoarBridge("/nonexistent/agent.soar")) {
      bad.start();
    } catch (IllegalStateException expected) {
      threw = true;
    }
    check(threw, "start() throws for a missing productions file");

    Recorder robot = new Recorder();
    try (SoarBridge soar = new SoarBridge(agents + "/default.soar")) {
      soar.start();
      check(soar.isRunning(), "running after start");

      soar.setMatch(135.0, MatchMode.Teleop, Alliance.Red);
      soar.setVision(true, 2.0, 10.0);
      for (int i = 0; i < 50 && robot.moves == 0; i++) {
        soar.sendInputs();
        Thread.sleep(2);
        soar.dispatchCommands(robot);
      }
      check(robot.moves == 1, "one move command, got " + robot.moves);
      check(robot.intakes == 1, "one run-intake command");
      check(Math.abs(robot.omega + 0.3) < 1e-9, "omega = -0.03 * yaw");

      soar.setIntakeHasPiece(true);
      for (int i = 0; i < 50 && robot.shots == 0; i++) {
        soar.sendInputs();
        Thread.sleep(2);
        soar.dispatchCommands(robot);
      }
      check(robot.shots >= 1, "shoot command after has-piece");

      // A short burst at loop rate to exercise allocation paths.
      for (int i = 0; i < 2000; i++) {
        soar.setMatch(135.0 - i * 0.02, MatchMode.Teleop, Alliance.Red);
        soar.setIntakeHasPiece((i / 50) % 2 == 0);
        soar.sendInputs();
        soar.dispatchCommands(robot);
        if (i % 10 == 0) {
          Thread.sleep(1);
        }
      }
      check(soar.commandsRejected() == 0, "no rejected commands");
      check(soar.inputsProcessed() > 100, "inputs processed");
      check(soar.maxLatencyMillis() < 5.0, "max latency < 5 ms, got " + soar.maxLatencyMillis());
      System.out.printf("  latency: mean %.3f ms, max %.3f ms%n",
          soar.meanLatencyMillis(), soar.maxLatencyMillis());
    }

    generic(args.length > 1 ? args[1] : agents + "/../tests");

    System.out.println(failures == 0 ? "PASSED" : "FAILED (" + failures + ")");
    System.exit(failures == 0 ? 0 : 1);
  }
}
