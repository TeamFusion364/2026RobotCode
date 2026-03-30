// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Creates simulated Shooter IO */
public class ShooterIOSim implements ShooterIO {

  // Shooter setup
  private FlywheelSim shooterSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(2),
              Constants.Shooter.FlywheelWeightKG,
              Constants.Shooter.shooterGearRatio),
          DCMotor.getKrakenX60(2));

  private double ShooterVelocityRPS = 0.0;
  private double ShooterAppliedVolts = 0.0;
  private double ShooterAppliedAmps = 0.0;
  private boolean closeShooterLoop = false;
  // Simulated shooter PID constants
  private ProfiledPIDController ShooterPID =
      new ProfiledPIDController(
          0.04,
          0.0,
          0.0,
          new Constraints(Constants.Shooter.shooterMaxAccel, Constants.Shooter.shooterMaxJerk));

  // Hood setup
  private SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          Constants.Shooter.hoodGearRatio,
          Units.inchesToMeters(4),
          Units.inchesToMeters(8.3),
          Units.degreesToRadians(0),
          Units.degreesToRadians(25),
          false,
          Units.degreesToRadians(0.01));

  private double HoodPosition = 0.0;
  private double HoodCurrentAmps = 0.0;
  // Simulated hood PID constants
  private PIDController HoodPID = new PIDController(0.05, 0, 0);

  // Turret setup
  private SingleJointedArmSim turretSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          Constants.Shooter.turretGearRatio,
          Units.inchesToMeters(2),
          Units.inchesToMeters(6.5),
          Units.degreesToRadians(Constants.Shooter.turretReverseLimit),
          Units.degreesToRadians(Constants.Shooter.turretForwardLimit),
          false,
          Units.degreesToRadians(0.01));

  private double TurretPosition = 0.0;
  private double TurretCurrentAmps = 0.0;
  // Simulated hood PID constants
  private PIDController TurretPID = new PIDController(0.3, 0, 0);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // inputs for shooter
    if (closeShooterLoop) {
      ShooterAppliedVolts = ShooterPID.calculate(shooterSim.getAngularVelocityRPM() * 60);
    }
    // Update simulation state
    shooterSim.setInput(MathUtil.clamp(ShooterAppliedVolts, -12.0, 12.0));
    shooterSim.update(0.02);

    inputs.ShooterVelocityRPS = (shooterSim.getAngularVelocityRPM() * 60);
    inputs.ShooterVelocityMPS =
        ((shooterSim.getAngularVelocityRPM() * 60)
            * Units.inchesToMeters(Constants.Shooter.shooterWheelDiameter * Math.PI));
    inputs.ShooterAppliedVolts = ShooterAppliedVolts;
    inputs.ShooterAppliedAmps = shooterSim.getCurrentDrawAmps();

    // inputs for hood
    double hoodVolts = HoodPID.calculate(Units.radiansToDegrees(hoodSim.getAngleRads()));
    hoodSim.setInput(MathUtil.clamp(hoodVolts, -12.0, 12.0));
    hoodSim.update(0.02);
    inputs.HoodPosition = Units.radiansToDegrees(hoodSim.getAngleRads());
    inputs.HoodCurrentAmps = hoodSim.getCurrentDrawAmps();

    // inputs for turret
    double turretVolts = TurretPID.calculate(Units.radiansToDegrees(turretSim.getAngleRads()));
    turretSim.setInput(MathUtil.clamp(turretVolts, -12.0, 12.0));
    turretSim.update(0.02);
    inputs.TurretPosition = Units.radiansToDegrees(turretSim.getAngleRads());
    inputs.TurretVelocity = Units.radiansToDegrees(turretSim.getVelocityRadPerSec());
    inputs.TurretCurrentAmps = turretSim.getCurrentDrawAmps();
    inputs.TurretAtSetpoint = true;
  }

  @Override
  public void setShooterVoltage(double Volts) {
    closeShooterLoop = false;
    ShooterAppliedVolts = Volts;
  }

  @Override
  public void setShooterRPS(double RPS) {
    closeShooterLoop = true;
    ShooterPID.setGoal(RPS);
  }

  public void setShooterMPS(double MPS) {
    closeShooterLoop = true;
    ShooterPID.setGoal(
        MPS / Units.inchesToMeters(Constants.Shooter.shooterWheelDiameter * Math.PI));
  }

  @Override
  public void setShooterHoodAngle(double Degrees) {
    HoodPID.setSetpoint(Degrees);
  }

  @Override
  public void setTurretVoltage(double Volts) {
    turretSim.setInput(Volts);
  }

  @Override
  public void setTurretAngle(double Degrees) {
    TurretPID.setSetpoint(Degrees);
  }

  @Override
  public void resetTurretZero(double Position) {
  }
}
