package frc.robot.subsystems.Feeder;

import org.littletonrobotics.junction.AutoLog;

/** Hopper IO interface for AdvantageKit logging and platform abstraction */
public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    // Feeder roller (named `Feeder` in Constants)
    public double FeederAppliedVolts = 0.0;
    public double FeederAppliedAmps = 0.0;
  }

  // Update inputs for logging
  public default void updateInputs(FeederIOInputs inputs) {}

  // Intake roller control
  public default void setFeederVoltage(double volts) {}
}
