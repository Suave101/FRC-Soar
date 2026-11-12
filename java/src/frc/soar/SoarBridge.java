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
 * <p>Any robot, by input-link path and command name instead of the fixed schema:
 *
 * <pre>{@code
 * // robotInit
 * soar.onCommand("ground-intake", c -> intake.setState(c.getString("state")));
 *
 * // robotPeriodic
 * soar.setBool("ground-intake.beam-break", !beamBreak.get());
 * soar.setString("match.mode", "teleop");
 * soar.sendInputs();
 * ...
 * soar.dispatchCommands();
 * }</pre>
 *
 * <p>The periodic methods never wait on Soar. All native memory is owned by this
 * object and released by {@link #close()}. Call from one thread (the robot loop).
 */
public final class SoarBridge implements AutoCloseable {
  private final SoarHardwareInterface hardware;
  private final RobotInputs inputs = new RobotInputs();
  private final java.util.Map<String, CommandListener> listeners = new java.util.HashMap<>();
  // Set once a typed setter (setMatch, setDrive, ...) is used; only then does
  // sendInputs() write the fixed schema, so path-only robots get none of it.
  private boolean typedInputs;
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
    typedInputs = true;
    inputs.setMatchTime(timeRemainingSeconds);
    inputs.setMode(mode);
    inputs.setAlliance(alliance);
  }

  public void setDrive(double xMeters, double yMeters, double headingDegrees,
                       double vxMetersPerSecond, double vyMetersPerSecond) {
    typedInputs = true;
    inputs.setPoseX(xMeters);
    inputs.setPoseY(yMeters);
    inputs.setHeading(headingDegrees);
    inputs.setVelocityX(vxMetersPerSecond);
    inputs.setVelocityY(vyMetersPerSecond);
  }

  public void setVision(boolean targetDetected, double distanceMeters, double yawDegrees) {
    typedInputs = true;
    inputs.setTargetDetected(targetDetected);
    inputs.setTargetDistance(distanceMeters);
    inputs.setTargetYaw(yawDegrees);
  }

  public void setIntakeHasPiece(boolean hasPiece) {
    typedInputs = true;
    inputs.setHasPiece(hasPiece);
  }

  public void setElevatorHeight(double heightMeters) {
    typedInputs = true;
    inputs.setElevatorHeight(heightMeters);
  }

  // --- Inputs by path, for any robot. Paths are relative to ^io.input-link:
  // "ground-intake.beam-break" is ^ground-intake <x>, <x> ^beam-break value.
  // Each returns false if the path is invalid or conflicts with an existing
  // one (see lastWarning()). Values reach the agent at the next sendInputs().

  /** Float WME. */
  public boolean setNumber(String path, double value) {
    checkOpen();
    return hardware.setNumber(path, value);
  }

  /** Int WME; a production testing {@code ^count 1} needs an int, not 1.0. */
  public boolean setInt(String path, long value) {
    checkOpen();
    return hardware.setInt(path, value);
  }

  public boolean setString(String path, String value) {
    checkOpen();
    return hardware.setString(path, value);
  }

  /** The string {@code true} or {@code false}. */
  public boolean setBool(String path, boolean value) {
    checkOpen();
    return hardware.setBool(path, value);
  }

  /** Removes a value, or an identifier and everything under it. */
  public boolean removeInput(String path) {
    checkOpen();
    return hardware.removeInput(path);
  }

  /** Publishes the input values set since the last call to the agent's input-link. */
  public void sendInputs() {
    checkOpen();
    if (typedInputs) {
      hardware.setInputs(inputs);  // typed fields, then the same publish
    } else {
      hardware.sendInputs();
    }
  }

  // --- Commands by name ---

  /**
   * Lets the agent issue output-link commands named {@code name} and calls
   * {@code listener} for each in {@link #dispatchCommands()}. Replaces an
   * earlier listener for the name. Commands with names nobody registered are
   * rejected by the bridge with {@code ^status error}; the built-in commands
   * (move, turn-to, set-elevator, run-intake, shoot) are always accepted.
   *
   * @throws IllegalArgumentException for an invalid name
   */
  public void onCommand(String name, CommandListener listener) {
    checkOpen();
    if (!hardware.acceptCommand(name)) {
      throw new IllegalArgumentException(hardware.getLastWarning());
    }
    listeners.put(name, listener);
  }

  /** Dispatches to {@link #onCommand} listeners only. */
  public int dispatchCommands() {
    return dispatch(null);
  }

  /**
   * Calls the {@link #onCommand} listener, or for a built-in command with no
   * listener the matching {@code actuators} method, for every command the
   * agent has issued since the last call, and reports each back to Soar as
   * {@code ^status complete}. A command whose handler throws is reported as
   * {@code ^status error}; the remaining commands are still dispatched and the
   * first exception is rethrown. A command with no handler is reported as
   * {@code ^status error} and counted in {@link #commandsUnhandled()}.
   *
   * @return the number of commands dispatched
   */
  public int dispatchCommands(RobotActuators actuators) {
    return dispatch(actuators);
  }

  private int dispatch(RobotActuators actuators) {
    checkOpen();
    CommandVector commands = hardware.pollCommands();
    RuntimeException failure = null;
    try {
      int count = commands.size();
      for (int i = 0; i < count; i++) {
        Command command = commands.get(i);
        try {
          CommandListener listener = listeners.get(command.getName());
          if (listener == null
              && (actuators == null || command.getType() == CommandType.Custom)) {
            hardware.rejectCommand(command.getId(),
                "no handler for ^" + command.getName() + " (register one with onCommand)");
            continue;
          }
          boolean ok = false;
          try {
            if (listener != null) {
              listener.execute(SoarCommand.from(command));
            } else {
              execute(command, actuators);
            }
            ok = true;
          } catch (RuntimeException e) {
            if (failure == null) {
              failure = e;
            }
          }
          hardware.completeCommand(command.getId(), ok);
        } finally {
          command.delete();
        }
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

  /** Commands discarded (and marked {@code ^status error}) because nobody dispatched them. */
  public long commandsDropped() {
    return (long) stat(Stat.COMMANDS_DROPPED);
  }

  /** Commands dispatched with no listener or actuator method for their name. */
  public long commandsUnhandled() {
    return (long) stat(Stat.COMMANDS_UNHANDLED);
  }

  /** Constant WMEs the agent put directly on the output-link; commands must be identifiers. */
  public long outputConstantsIgnored() {
    return (long) stat(Stat.OUTPUT_CONSTANTS_IGNORED);
  }

  /** Path setter calls refused for an invalid or conflicting path. */
  public long inputErrors() {
    return (long) stat(Stat.INPUT_ERRORS);
  }

  /** Most recent non-fatal problem (refused path, rejected command, ...), or "". */
  public String lastWarning() {
    checkOpen();
    return hardware.getLastWarning();
  }

  private enum Stat {
    MEAN_LATENCY, MAX_LATENCY, INPUTS_PROCESSED, COMMANDS_REJECTED, COMMANDS_DROPPED,
    COMMANDS_UNHANDLED, OUTPUT_CONSTANTS_IGNORED, INPUT_ERRORS
  }

  private double stat(Stat which) {
    checkOpen();
    BridgeStats stats = hardware.getStats();
    try {
      switch (which) {
        case MEAN_LATENCY: return stats.getMeanLatencyMicros();
        case MAX_LATENCY: return stats.getMaxLatencyMicros();
        case INPUTS_PROCESSED: return stats.getInputsProcessed();
        case COMMANDS_DROPPED: return stats.getCommandsDropped();
        case COMMANDS_UNHANDLED: return stats.getCommandsUnhandled();
        case OUTPUT_CONSTANTS_IGNORED: return stats.getOutputConstantsIgnored();
        case INPUT_ERRORS: return stats.getInputErrors();
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
