package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem: roller + linear extension (in the style of Shooter subsystem) */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  // Roller controls
  // Intake roller controls
  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  public void setIntakeRPS(double rps) {
    io.setIntakeRPS(rps);
  }

  // Extension controls (linear)
  // Stroker (linear deployer) controls (inches)
  public void setStrokerVoltage(double volts) {
    io.setStrokerVoltage(volts);
  }

  // inches from 0 - 12
  public void setStrokerPositionInches(double inches) {
    io.setStrokerPosition(inches);
  }

  // Getters
  @AutoLogOutput(key = "Intake/IntakeAppliedVolts")
  public double getIntakeAppliedVolts() {
    return inputs.IntakeAppliedVolts;
  }

  public double getIntakeRPS() {
    return inputs.IntakeVelocityRPS;
  }

  @AutoLogOutput(key = "Intake/StrokerPositionInches")
  public double getStrokerPositionInches() {
    return inputs.StrokerPositionInches;
  }

  @AutoLogOutput(key = "Intake/StrokerAppliedVolts")
  public double getStrokerAppliedVolts() {
    return inputs.StrokerAppliedVolts;
  }

  @AutoLogOutput(key = "Intake/StrokerAppliedAmps")
  public double getStrokerAppliedAmps() {
    return inputs.StrokerAppliedAmps;
  }

  @AutoLogOutput(key = "Intake/isStrokerAtSetpoint")
  public boolean isStrokerAtSetpoint() {
    double targetPosition = inputs.StrokerPositionInches;
    double currentPosition = getStrokerPositionInches();
    return Math.abs(currentPosition - targetPosition) <= 0.25;
  }
}
