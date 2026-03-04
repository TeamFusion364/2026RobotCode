package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Climber subsystem */
public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  // Track the current command running on this subsystem
  private String currentCommandName = "None";

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    // Log the current command running on this subsystem
    Command currentCommand = getCurrentCommand();
    currentCommandName = (currentCommand != null) ? currentCommand.getName() : "None";
    Logger.recordOutput("Climber/CurrentCommand", currentCommandName);
  }

  // Extension controls (linear)
  public void setClimberVoltage(double volts) {
    io.setClimberVoltage(volts);
  }

  // rotations from 0-113
  public void setClimberPosition(double rotations) {
    io.setClimberPosition(rotations);
  }

  // Getters
  @AutoLogOutput(key = "Climber/ClimberAppliedVolts")
  public double getClimberAppliedVolts() {
    return inputs.ClimberAppliedVolts;
  }

  @AutoLogOutput(key = "Climber/ClimberPosition")
  public double getClimberPosition() {
    return inputs.ClimberPosition;
  }

  @AutoLogOutput(key = "Climber/ClimberAppliedAmps")
  public double getClimberAppliedAmps() {
    return inputs.ClimberAppliedAmps;
  }

  @AutoLogOutput(key = "Climber/isClimberAtSetpoint")
  public boolean isClimberAtSetpoint(double setpoint) {
    double currentPosition = getClimberPosition();
    return Math.abs(currentPosition - setpoint) <= Constants.Climber.climberDeadband;
  }
}
