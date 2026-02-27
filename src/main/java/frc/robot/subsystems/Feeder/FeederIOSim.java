package frc.robot.subsystems.Feeder;

/** Sim implementation for Feeder using WPILib's ElevatorSim mapped to a drum radius. */
public class FeederIOSim implements FeederIO {
  // Roller simple model
  private double feederAppliedVolts = 0.0;

  public FeederIOSim() {}

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Feeder roller
    inputs.FeederAppliedVolts = feederAppliedVolts;
    inputs.FeederAppliedAmps = Math.abs(feederAppliedVolts) * 0.1;
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederAppliedVolts = volts;
  }
}
