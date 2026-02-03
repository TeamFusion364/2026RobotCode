// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Creates a new turret subsystem. Advantagekit/sim compatible */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();

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
    Alliance alliance = DriverStation.getAlliance().get();
    Distance blueZone = Meters.of(4.6);
    Distance redZone = Meters.of(11.9);

    if (robotPose.getMeasureX().isNear(redZone, Meters.of(1))
        || robotPose.getMeasureX().isNear(blueZone, Meters.of(1))) {
      return true;
    }
    return false;
  }
}
