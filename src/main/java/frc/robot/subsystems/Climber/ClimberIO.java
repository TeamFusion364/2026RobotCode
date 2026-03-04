package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

/** Climber IO interface for AdvantageKit logging and platform abstraction */
public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {

    // Linear deployer (named `stroker` in Constants) - position reported in inches
    public double ClimberPosition = 0.0;
    public double ClimberAppliedVolts = 0.0;
    public double ClimberAppliedAmps = 0.0;
  }

  // Update inputs for logging
  public default void updateInputs(ClimberIOInputs inputs) {}

  // Climber voltage control
  public default void setClimberVoltage(double volts) {}

  // Climber positional control (rotations)
  public default void setClimberPosition(double rotations) {}
}
