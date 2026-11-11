import frc.soar.Alliance;
import frc.soar.MatchMode;
import frc.soar.RobotActuators;
import frc.soar.SoarBridge;

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

    System.out.println(failures == 0 ? "PASSED" : "FAILED (" + failures + ")");
    System.exit(failures == 0 ? 0 : 1);
  }
}
