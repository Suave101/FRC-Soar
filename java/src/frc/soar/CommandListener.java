package frc.soar;

/**
 * Handles one named output-link command; register with
 * {@link SoarBridge#onCommand(String, CommandListener)}. Runs on the thread
 * that calls {@link SoarBridge#dispatchCommands()}. Throwing marks the command
 * {@code ^status error}.
 */
@FunctionalInterface
public interface CommandListener {
  void execute(SoarCommand command);
}
