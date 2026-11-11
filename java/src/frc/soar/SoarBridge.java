package frc.soar;

/**
 * Runs a Soar agent on a background thread and connects it to the robot.
 *
 * <pre>{@code
 * // robotInit
 * SoarBridge soar = new SoarBridge(Filesystem.getDeployDirectory() + "/robot.soar");
 * soar.start();
 *
 * // robotPeriodic
 * soar.setMatch(DriverStation.getMatchTime(), MatchMode.Teleop, Alliance.Blue);
 * soar.setDrive(pose.getX(), pose.getY(), gyro.getAngle(), speeds.vx, speeds.vy);
 * soar.sendInputs();
 * ...
 * soar.dispatchCommands(actuators);
 * }</pre>
 *
 * <p>The periodic methods never wait on Soar. All native memory is owned by this
 * object and released by {@link #close()}.
 */
public final class SoarBridge implements AutoCloseable {
  private final SoarHardwareInterface hardware;
  private final RobotInputs inputs = new RobotInputs();
  private boolean closed;

  /** @param productionsPath .soar file to load when {@link #start()} is called */
  public SoarBridge(String productionsPath) {
    this(productionsPath, 1, 0);
  }

  /**
   * @param productionsPath .soar file to load when {@link #start()} is called
   * @param decisionsPerInput Soar decision cycles to run per {@link #sendInputs()}
   * @param debuggerPort TCP port for the Soar debugger, or 0 for none
   */
  public SoarBridge(String productionsPath, int decisionsPerInput, int debuggerPort) {
    this(productionsPath, decisionsPerInput, debuggerPort, 0);
  }

  /**
   * @param productionsPath .soar file to load when {@link #start()} is called
   * @param decisionsPerInput Soar decision cycles to run per {@link #sendInputs()}
   * @param debuggerPort TCP port for the Soar debugger, or 0 for none
   * @param realtimePriority SCHED_FIFO priority (1-99) for the Soar thread, or 0
   *     for normal scheduling; use a value below the robot loop's priority
   */
  public SoarBridge(String productionsPath, int decisionsPerInput, int debuggerPort,
                    int realtimePriority) {
    BridgeConfig config = new BridgeConfig();
    try {
      config.setProductionsPath(productionsPath);
      config.setDecisionsPerInput(decisionsPerInput);
      config.setDebuggerPort(debuggerPort);
      config.setRealtimePriority(realtimePriority);
      hardware = new SoarHardwareInterface(config);
    } finally {
      config.delete();
    }
  }

  /**
   * Starts the Soar thread and loads the productions. Takes tens of
   * milliseconds; call from robotInit.
   *
   * @throws IllegalStateException if the kernel or productions fail to load
   */
  public void start() {
    checkOpen();
    if (!hardware.start()) {
      throw new IllegalStateException(hardware.getLastError());
    }
  }

  public boolean isRunning() {
    return !closed && hardware.isRunning();
  }

  // --- Inputs: set whichever groups changed, then sendInputs() once per loop. ---

  public void setMatch(double timeRemainingSeconds, MatchMode mode, Alliance alliance) {
    inputs.setMatchTime(timeRemainingSeconds);
    inputs.setMode(mode);
    inputs.setAlliance(alliance);
  }

  public void setDrive(double xMeters, double yMeters, double headingDegrees,
                       double vxMetersPerSecond, double vyMetersPerSecond) {
    inputs.setPoseX(xMeters);
    inputs.setPoseY(yMeters);
    inputs.setHeading(headingDegrees);
    inputs.setVelocityX(vxMetersPerSecond);
    inputs.setVelocityY(vyMetersPerSecond);
  }

  public void setVision(boolean targetDetected, double distanceMeters, double yawDegrees) {
    inputs.setTargetDetected(targetDetected);
    inputs.setTargetDistance(distanceMeters);
    inputs.setTargetYaw(yawDegrees);
  }

  public void setIntakeHasPiece(boolean hasPiece) {
    inputs.setHasPiece(hasPiece);
  }

  public void setElevatorHeight(double heightMeters) {
    inputs.setElevatorHeight(heightMeters);
  }

  /** Publishes the current input values to the agent's input-link. */
  public void sendInputs() {
    checkOpen();
    hardware.setInputs(inputs);
  }

  /**
   * Calls {@code actuators} for every command the agent has issued since the
   * last call and reports each back to Soar as {@code ^status complete}. A
   * command whose handler throws is reported as {@code ^status error}; the
   * remaining commands are still dispatched and the first exception is rethrown.
   *
   * @return the number of commands dispatched
   */
  public int dispatchCommands(RobotActuators actuators) {
    checkOpen();
    CommandVector commands = hardware.pollCommands();
    RuntimeException failure = null;
    try {
      int count = commands.size();
      for (int i = 0; i < count; i++) {
        Command command = commands.get(i);
        boolean ok = false;
        try {
          execute(command, actuators);
          ok = true;
        } catch (RuntimeException e) {
          if (failure == null) {
            failure = e;
          }
        }
        hardware.completeCommand(command.getId(), ok);
      }
      if (failure != null) {
        throw failure;
      }
      return count;
    } finally {
      commands.delete();
    }
  }

  private static void execute(Command command, RobotActuators actuators) {
    switch (command.getType()) {
      case Move:
        actuators.move(command.getVx(), command.getVy(), command.getOmega());
        break;
      case TurnTo:
        actuators.turnTo(command.getHeading());
        break;
      case SetElevator:
        actuators.setElevator(command.getHeight());
        break;
      case RunIntake:
        actuators.runIntake(command.getSpeed());
        break;
      case Shoot:
        actuators.shoot(command.getExecute());
        break;
      default:
        throw new IllegalStateException("Unhandled command type " + command.getType());
    }
  }

  /** Mean time from {@link #sendInputs()} to the resulting commands being available. */
  public double meanLatencyMillis() {
    return stat(Stat.MEAN_LATENCY) / 1000.0;
  }

  /** Worst time from {@link #sendInputs()} to the resulting commands being available. */
  public double maxLatencyMillis() {
    return stat(Stat.MAX_LATENCY) / 1000.0;
  }

  /** Input snapshots the agent has processed. */
  public long inputsProcessed() {
    return (long) stat(Stat.INPUTS_PROCESSED);
  }

  /** Commands rejected for an unknown name or a missing or malformed parameter. */
  public long commandsRejected() {
    return (long) stat(Stat.COMMANDS_REJECTED);
  }

  private enum Stat { MEAN_LATENCY, MAX_LATENCY, INPUTS_PROCESSED, COMMANDS_REJECTED }

  private double stat(Stat which) {
    checkOpen();
    BridgeStats stats = hardware.getStats();
    try {
      switch (which) {
        case MEAN_LATENCY: return stats.getMeanLatencyMicros();
        case MAX_LATENCY: return stats.getMaxLatencyMicros();
        case INPUTS_PROCESSED: return stats.getInputsProcessed();
        default: return stats.getCommandsRejected();
      }
    } finally {
      stats.delete();
    }
  }

  /** Stops the Soar thread and frees all native memory. Safe to call twice. */
  @Override
  public void close() {
    if (closed) {
      return;
    }
    closed = true;
    hardware.stop();
    hardware.delete();
    inputs.delete();
  }

  private void checkOpen() {
    if (closed) {
      throw new IllegalStateException("SoarBridge is closed");
    }
  }
}
