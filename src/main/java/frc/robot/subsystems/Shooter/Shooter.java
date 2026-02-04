// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Creates a new turret subsystem. Advantagekit/sim compatible */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();

  // Optional supplier for the current robot pose (provided by RobotContainer/Drive)
  private Supplier<Pose2d> robotPoseSupplier = null;

  public Shooter(ShooterIO io) {
    this.io = io;

    // Initiation code goes here

    // Shootermap config
    for (double[] row : Constants.ShooterMaps.HubMap) {
      angleMap.put(row[0], row[1]);
      velocityMap.put(row[0], row[2]);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  // Set shooter raw voltage
  public void setShooterVoltage(double Volts) {
    io.setShooterVoltage(Volts);
  }

  // Set shooter velocity in RPS
  public void setShooterRPS(double RPS) {
    io.setShooterRPS(RPS);
  }

  // Set shooter velocity in MPS
  public void setShooterMPS(double MPS) {
    io.setShooterMPS(MPS);
  }

  // Set hood angle in Degrees
  public void setHoodAngle(double Degrees) {
    io.setShooterHoodAngle(Degrees);
  }

  // Set turret angle in Degrees
  public void setTurretAngle(double Degrees) {
    io.setTurretAngle(Degrees);
  }

  // Set turret raw voltage
  public void setTurretVoltage(double Volts) {
    io.setTurretVoltage(Volts);
  }

  public double getTurretAngle() {
    return inputs.TurretPosition;
  }

  // Returns the current measured shooter wheel velocity in RPS
  public double getShooterRPS() {
    return inputs.ShooterVelocityRPS;
  }

  /**
   * Compute the smart target pose used by the previous SmartTrack command. This encapsulates the
   * alliance/flip logic so callers don't duplicate it.
   *
   * @param robotPose current robot pose
   * @return the target Pose2d to aim at (hub center or feeding target)
   */
  public Pose2d getSmartTargetPose(Pose2d robotPose) {
    Pose2d targetPose;
    if (isInAllianceZone(robotPose)) {
      if (getShooterFlipped()) {
        targetPose = FieldConstants.RedHubCenter;
      } else {
        targetPose = FieldConstants.BlueHubCenter;
      }
    } else {
      if (!getShooterFlipped()) {
        targetPose = FieldConstants.RedFeedingTarget;
      } else {
        targetPose = FieldConstants.BlueFeedingTarget;
      }
    }

    return targetPose;
  }

  /**
   * Calculate the mapped shooter RPS for the "smart" target selection given the robot pose. This
   * encapsulates the math currently used by the flywheel setpoint trigger.
   *
   * @param robotPose current robot pose
   * @return target shooter velocity in RPS (mapped from distance)
   */
  public double CalculateShooterRPS(Pose2d robotPose) {
    // Determine target pose using existing smart target selection
    Pose2d targetPose = getSmartTargetPose(robotPose);

    // Shooter position in field coordinates
    Pose2d shooterPositionPose = robotPose.plus(Constants.Shooter.shooterOffset);

    // Shooter heading = robot heading + turret angle
    Rotation2d shooterHeading =
        robotPose.getRotation().plus(Rotation2d.fromDegrees(this.getTurretAngle()));

    // Final shooter pose
    Pose2d shooterPose = new Pose2d(shooterPositionPose.getTranslation(), shooterHeading);

    // Distance from turret to target
    double shotDistanceMeters = calculateDistanceFromGoal(shooterPose, targetPose);

    // Map distance to RPS
    double targetRPS = getMappedVelocity(shotDistanceMeters);
    return targetRPS;
  }

  // Check to see if path should be flipped.
  @AutoLogOutput(key = "Shooter/getFlipped")
  public boolean getShooterFlipped() {
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isFlipped;
  }

  public double calculateTurretAngleToGoal(Pose2d turretPose, Pose2d targetPose) {
    double outputDegrees;

    Translation2d delta = targetPose.getTranslation().minus(turretPose.getTranslation());
    Rotation2d angleToTarget = new Rotation2d(delta.getX(), delta.getY());

    double turretRotationGoal = angleToTarget.getDegrees();

    outputDegrees = turretRotationGoal;
    if (outputDegrees < 0) {
      outputDegrees = outputDegrees + 360;
    }

    return outputDegrees;
  }

  public double calculateDistanceFromGoal(Pose2d turretPose, Pose2d targetPose) {
    double outputMeters;
    outputMeters = targetPose.getTranslation().getDistance(turretPose.getTranslation());
    return Math.abs(outputMeters);
  }

  // Returns interpolated hood angle value
  public double getMappedHoodAngle(double distanceMeters) {
    return angleMap.get(distanceMeters);
  }

  // Returns interpolated shooter speed value
  public double getMappedVelocity(double distanceMeters) {
    return velocityMap.get(distanceMeters);
  }

  @AutoLogOutput(key = "Shooter/isInAllianceZone")
  public boolean isInAllianceZone(Pose2d robotPose) {
    // DriverStation alliance may not be available yet during early initialization.
    if (!DriverStation.getAlliance().isPresent()) {
      return false;
    }

    Alliance alliance = DriverStation.getAlliance().get();
    Distance blueZone = Inches.of(182);
    Distance redZone = Inches.of(469);

    if (alliance == Alliance.Blue && robotPose.getMeasureX().lt(blueZone)) {
      return true;
    } else if (alliance == Alliance.Red && robotPose.getMeasureX().gt(redZone)) {
      return true;
    }

    return false;
  }

  @AutoLogOutput(key = "Shooter/isInTrench")
  public boolean isInTrench(Pose2d robotPose) {
    // If alliance data isn't available yet, be conservative and report not-in-trench
    if (!DriverStation.getAlliance().isPresent()) {
      return false;
    }

    Distance blueZone = Meters.of(4.6);
    Distance redZone = Meters.of(11.9);

    if (robotPose.getMeasureX().isNear(redZone, Meters.of(1))
        || robotPose.getMeasureX().isNear(blueZone, Meters.of(1))) {
      return true;
    }
    return false;
  }

  /**
   * Provide a supplier for the current robot pose. This allows the shooter to build triggers or
   * internal logic that depend on the robot pose without pulling Drive into RobotContainer.
   */
  public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    this.robotPoseSupplier = supplier;
  }

  /**
   * Returns the current robot pose via the configured supplier, or a default Pose2d if the supplier
   * hasn't been configured yet. This provides a safe single call-site for other code to obtain the
   * robot pose without carrying a Drive reference.
   */
  public Pose2d getRobotPose() {
    if (this.robotPoseSupplier == null) {
      return new Pose2d();
    }
    return this.robotPoseSupplier.get();
  }

  /**
   * Returns a Trigger that becomes true when the current measured flywheel RPS is within tolerance
   * of the mapped RPS for the smart target computed from the provided robot pose. The robot pose
   * supplier must have been provided via {@link #setRobotPoseSupplier}.
   */
  public Trigger getFlywheelAtSetpointTrigger(double toleranceRPS) {
    return new Trigger(
        () -> {
          if (robotPoseSupplier == null) {
            return false;
          }

          Pose2d robotPose = robotPoseSupplier.get();
          double targetRPS = CalculateShooterRPS(robotPose);
          double currentRPS = getShooterRPS();
          return Math.abs(currentRPS - targetRPS) <= toleranceRPS;
        });
  }

  // Flywheel SysID Setup
  public SysIdRoutine flywheelRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("FlywheelRoutineTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> this.setShooterVoltage(voltage.in(Volts)), null, this));

  // Flywheel SysID Routines
  public Command flywheelQuasistatic(SysIdRoutine.Direction direction) {
    return flywheelRoutine.quasistatic(direction);
  }

  public Command flywheelDynamic(SysIdRoutine.Direction direction) {
    return flywheelRoutine.dynamic(direction);
  }
}
