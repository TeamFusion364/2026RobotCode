package frc.robot.subsystems.Kicker;

import org.littletonrobotics.junction.AutoLog;

/** Hopper IO interface for AdvantageKit logging and platform abstraction */
public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    // Kicker roller (named `Kicker` in Constants)
    public double KickerAppliedVolts = 0.0;
    public double KickerAppliedAmps = 0.0;
  }

  // Update inputs for logging
  public default void updateInputs(KickerIOInputs inputs) {}

  // Intake roller control
  public default void setKickerVoltage(double volts) {}
}
