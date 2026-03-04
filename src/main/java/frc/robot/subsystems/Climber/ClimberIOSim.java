package frc.robot.subsystems.Climber;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

/** Sim implementation for Intake using WPILib's ElevatorSim mapped to a drum radius. */
public class ClimberIOSim implements ClimberIO {

  private double climberAppliedVolts = 0.0;
  private double climberMinMeters;
  private double climberMaxMeters;
  private double climberPosition;

  public ClimberIOSim() {
    climberMinMeters = Constants.Climber.climberReverseLimit;
    climberMaxMeters = Constants.Climber.climberForwardLimit;
  }

  @Override
  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {

    double positionMeters = climberPosition;
    // Convert meters -> inches for public API
    inputs.ClimberPosition = positionMeters;
    inputs.ClimberAppliedVolts = climberAppliedVolts;
    // Use sim current draw if available
    inputs.ClimberAppliedAmps = Math.abs(climberAppliedVolts) * 0.1;
  }

  @Override
  public void setClimberVoltage(double volts) {
    climberAppliedVolts = volts;
    climberPosition += climberAppliedVolts * 0.6; // Simple integration for simulation purposes
  }

  @Override
  public void setClimberPosition(double rotations) {
    PIDController climberController =
        new PIDController(
            Constants.Climber.climberKp, Constants.Climber.climberKi, Constants.Climber.climberKd);

    climberPosition += climberController.calculate(climberPosition, rotations);
  }
}
