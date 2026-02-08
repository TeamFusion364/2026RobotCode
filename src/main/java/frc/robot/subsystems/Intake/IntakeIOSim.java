package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Sim implementation for Intake using WPILib's ElevatorSim mapped to a drum radius. */
public class IntakeIOSim implements IntakeIO {
  // Roller simple model
  private double intakeAppliedVolts = 0.0;
  private double intakeVelocityRPS = 0.0;

  // We'll model the stroker as a drum/elevator. Derive the drum radius from constants.
  private final ElevatorSim strokerSim;
  private double strokerAppliedVolts = 0.0;
  private final double strokerMinMeters;
  private final double strokerMaxMeters;
  private final double drumRadiusMeters;

  public IntakeIOSim() {
    // Convert stroker limits (Constants are in inches) -> meters
    double fwdLimitInches = Constants.intake.strokerForwardLimit;
    double revLimitInches = Constants.intake.strokerReverseLimit;
    strokerMinMeters = Units.inchesToMeters(Math.min(fwdLimitInches, revLimitInches));
    strokerMaxMeters = Units.inchesToMeters(Math.max(fwdLimitInches, revLimitInches));

    // Derive drum radius from the configured gear diameter (assumed inches)
    drumRadiusMeters = Units.inchesToMeters(Constants.intake.strokerGearDiameter) / 2.0;

    // Gearing from Constants
    double gearing = Constants.intake.strokerGearRatio;

    // Carriage mass (not present in Constants) - pick a small default; can be tuned.
    double carriageMassKg = 1.0;

    // Create a linear elevator plant and then an ElevatorSim. The LinearSystemId helper
    // constructs the physics model; ElevatorSim wraps it for simulation.
    // Use the DCMotor-based ElevatorSim constructor. WPILib's signature requires
    // simulateGravity and a starting height after the gearing/mass/drum/min/max args.
    boolean simulateGravity = false; // drum/linear stroker doesn't need gravity by default
    double startingHeightMeters = 0; // start retracted
    strokerSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(1),
            gearing,
            carriageMassKg,
            drumRadiusMeters,
            strokerMinMeters,
            strokerMaxMeters,
            simulateGravity,
            startingHeightMeters);
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    // Intake roller
    inputs.IntakeAppliedVolts = intakeAppliedVolts;
    inputs.IntakeAppliedAmps = Math.abs(intakeAppliedVolts) * 0.1;
    inputs.IntakeVelocityRPS = intakeVelocityRPS;

    // Extension: apply voltage to the simulated elevator and integrate
    strokerSim.setInput(MathUtil.clamp(strokerAppliedVolts, -12.0, 12.0));
    strokerSim.update(0.02);
    double positionMeters = strokerSim.getPositionMeters();
    // Convert meters -> inches for public API
    inputs.StrokerPositionInches = Units.metersToInches(positionMeters);
    inputs.StrokerAppliedVolts = strokerAppliedVolts;
    // Use sim current draw if available
    inputs.StrokerAppliedAmps = strokerSim.getCurrentDrawAmps();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeAppliedVolts = volts;
  }

  @Override
  public void setIntakeRPS(double rps) {
    intakeVelocityRPS = rps;
    intakeAppliedVolts = MathUtil.clamp(rps * 0.01, -12.0, 12.0);
  }

  @Override
  public void setStrokerVoltage(double volts) {
    strokerAppliedVolts = volts;
  }

  @Override
  public void setStrokerPosition(double inches) {
    // Convert requested linear inches -> meters, then control toward that position
    double meters = Units.inchesToMeters(inches);
    double error = meters - strokerSim.getPositionMeters();
    // Simple P mapping from meters error to volts (tunable)
    strokerAppliedVolts = MathUtil.clamp(error * 12.0, -12.0, 12.0);
  }
}
