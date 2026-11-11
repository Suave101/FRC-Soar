package frc.soar;

/**
 * Receives the Soar agent's output-link commands on the robot thread.
 * Implement this by forwarding each call to the matching subsystem.
 */
public interface RobotActuators {
  /** {@code ^move}: field velocities in m/s and rotation in rad/s. */
  void move(double vx, double vy, double omega);

  /** {@code ^turn-to}: absolute heading in degrees. */
  void turnTo(double headingDegrees);

  /** {@code ^set-elevator}: target height in meters. */
  void setElevator(double heightMeters);

  /** {@code ^run-intake}: motor percent output, -1.0 to 1.0. */
  void runIntake(double speed);

  /** {@code ^shoot}: trigger (or cancel) the shooting sequence. */
  void shoot(boolean execute);
}
