package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Intake IO interface for AdvantageKit logging and platform abstraction */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Intake roller (named `intake` in Constants)
    public double IntakeAppliedVolts = 0.0;
    public double IntakeAppliedAmps = 0.0;
    public double IntakeVelocityRPS = 0.0;

    // Linear deployer (named `stroker` in Constants) - position reported in inches
    public double StrokerPositionInches = 0.0;
    public double StrokerAppliedVolts = 0.0;
    public double StrokerAppliedAmps = 0.0;
  }

  // Update inputs for logging
  public default void updateInputs(IntakeIOInputs inputs) {}

  // Intake roller control
  public default void setIntakeVoltage(double volts) {}

  public default void setIntakeRPS(double rps) {}

  // Stroker (linear deployer) control
  public default void setStrokerVoltage(double volts) {}

  // Set stroker position in inches
  public default void setStrokerPosition(double inches) {}

  public default void resetIntakeZero() {}
  ;
}
