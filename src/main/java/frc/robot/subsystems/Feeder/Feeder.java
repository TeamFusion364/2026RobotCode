package frc.robot.subsystems.Feeder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Feeder subsystem */
public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  // Track the current command running on this subsystem
  private String currentCommandName = "None";

  // Jam detection
  private final Debouncer jamDebouncer = new Debouncer(Constants.feeder.jamDetectionTimeThreshold);

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    // Log the current command running on this subsystem
    Command currentCommand = getCurrentCommand();
    currentCommandName = (currentCommand != null) ? currentCommand.getName() : "None";
    Logger.recordOutput("Feeder/CurrentCommand", currentCommandName);

    // Update jam detection
    boolean isJammed =
        jamDebouncer.calculate(getFeederAppliedAmps() > Constants.feeder.jamCurrentThreshold);
    Logger.recordOutput("Feeder/IsJammed", isJammed);
  }

  // Roller controls
  public void setFeederVoltage(double volts) {
    io.setFeederVoltage(volts);
  }

  // Getters
  @AutoLogOutput(key = "Feeder/AppliedVolts")
  public double getFeederAppliedVolts() {
    return inputs.FeederAppliedVolts;
  }

  @AutoLogOutput(key = "Feeder/AppliedAmps")
  public double getFeederAppliedAmps() {
    return inputs.FeederAppliedAmps;
  }

  @AutoLogOutput(key = "Feeder/JamDetected")
  public boolean isJammed() {
    return jamDebouncer.calculate(getFeederAppliedAmps() > Constants.feeder.jamCurrentThreshold);
  }

  public Trigger getFeederJammedTrigger() {
    return new Trigger(this::isJammed);
  }
}
