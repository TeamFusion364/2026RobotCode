package frc.robot.subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Sim implementation for Intake using WPILib's ElevatorSim mapped to a drum radius. */
public class ClimberIOSim implements ClimberIO {

  // We'll model the climber as a drum/elevator. Derive the drum radius from constants.
  private final ElevatorSim climberSim;
  private double climberAppliedVolts = 0.0;
  private final double climberMinMeters;
  private final double climberMaxMeters;
  private final double drumRadiusMeters;

  public ClimberIOSim() {
    climberMinMeters = Constants.Climber.climberReverseLimit;
    climberMaxMeters = Constants.Climber.climberForwardLimit;

    // Derive drum radius from the configured gear diameter (assumed inches)
    drumRadiusMeters = 1;

    // Gearing from Constants
    double gearing = 1;

    // Carriage mass (not present in Constants) - pick a small default; can be tuned.
    double carriageMassKg = 1.0;

    // Create a linear elevator plant and then an ElevatorSim. The LinearSystemId helper
    // constructs the physics model; ElevatorSim wraps it for simulation.
    // Use the DCMotor-based ElevatorSim constructor. WPILib's signature requires
    // simulateGravity and a starting height after the gearing/mass/drum/min/max args.
    boolean simulateGravity = false; // drum/linear stroker doesn't need gravity by default
    double startingHeightMeters = 0; // start retracted
    climberSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(1),
            gearing,
            carriageMassKg,
            drumRadiusMeters,
            climberMinMeters,
            climberMaxMeters,
            simulateGravity,
            startingHeightMeters);
  }

  @Override
  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {

    // Extension: apply voltage to the simulated elevator and integrate
    climberSim.setInput(MathUtil.clamp(climberAppliedVolts, -12.0, 12.0));
    climberSim.update(0.02);
    double positionMeters = climberSim.getPositionMeters();
    // Convert meters -> inches for public API
    inputs.ClimberPosition = positionMeters;
    inputs.ClimberAppliedVolts = climberAppliedVolts;
    // Use sim current draw if available
    inputs.ClimberAppliedAmps = climberSim.getCurrentDrawAmps();
  }

  @Override
  public void setClimberVoltage(double volts) {
    climberAppliedVolts = volts;
  }

  @Override
  public void setClimberPosition(double meters) {
    double error = meters - climberSim.getPositionMeters();
    // Simple P mapping from meters error to volts (tunable)
    climberAppliedVolts = MathUtil.clamp(error * 12.0, -12.0, 12.0);
  }
}
