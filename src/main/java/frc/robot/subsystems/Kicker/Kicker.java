package frc.robot.subsystems.Kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Kicker subsystem */
public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  // Track the current command running on this subsystem
  private String currentCommandName = "None";

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
}
