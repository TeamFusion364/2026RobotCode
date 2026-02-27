package frc.robot.subsystems.Kicker;

/** Sim implementation for Feeder using WPILib's ElevatorSim mapped to a drum radius. */
public class KickerIOSim implements KickerIO {
  // Roller simple model
  private double kickerAppliedVolts = 0.0;

  public KickerIOSim() {}

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    // Feeder roller
    inputs.KickerAppliedVolts = kickerAppliedVolts;
    inputs.KickerAppliedAmps = Math.abs(kickerAppliedVolts) * 0.1;
  }

  @Override
  public void setKickerVoltage(double volts) {
    kickerAppliedVolts = volts;
  }
}
