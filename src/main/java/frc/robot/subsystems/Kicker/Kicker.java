package frc.robot.subsystems.Kicker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Kicker subsystem */
public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  // Track the current command running on this subsystem
  private String currentCommandName = "None";

  // Jam detection
  private final Debouncer jamDebouncer = new Debouncer(Constants.kicker.jamDetectionTimeThreshold);

  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);

    // Log the current command running on this subsystem
    Command currentCommand = getCurrentCommand();
    currentCommandName = (currentCommand != null) ? currentCommand.getName() : "None";
    Logger.recordOutput("Kicker/CurrentCommand", currentCommandName);

    // Update jam detection
    boolean isJammed =
        jamDebouncer.calculate(getKickerAppliedAmps() > Constants.kicker.jamCurrentThreshold);
    Logger.recordOutput("Kicker/IsJammed", isJammed);
  }

  // Roller controls
  public void setKickerVoltage(double volts) {
    io.setKickerVoltage(volts);
  }

  // Getters
  @AutoLogOutput(key = "Kicker/AppliedVolts")
  public double getKickerAppliedVolts() {
    return inputs.KickerAppliedVolts;
  }

  @AutoLogOutput(key = "Kicker/AppliedAmps")
  public double getKickerAppliedAmps() {
    return inputs.KickerAppliedAmps;
  }

  @AutoLogOutput(key = "Kicker/JamDetected")
  public boolean isJammed() {
    return jamDebouncer.calculate(getKickerAppliedAmps() > Constants.kicker.jamCurrentThreshold);
  }

  public Trigger getKickerJammedTrigger() {
    return new Trigger(this::isJammed);
  }
}
