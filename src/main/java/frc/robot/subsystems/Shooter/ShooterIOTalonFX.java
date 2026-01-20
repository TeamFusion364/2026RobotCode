// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Robot;

/** TalonFX ShooterIO */
public class ShooterIOTalonFX implements ShooterIO {

  // Create flywheel motors
  private final TalonFX leaderMotor = new TalonFX(Constants.Shooter.leaderMotorID);
  private final TalonFX followerMotor = new TalonFX(Constants.Shooter.followerMotorID);
  // Create hood motor
  private final TalonFX hoodMotor = new TalonFX(Constants.Shooter.hoodMotorID);
  // Create turret motor
  private final TalonFX turretMotor = new TalonFX(Constants.Shooter.turretMotorID);

  // Declare shooter status signals
  private final StatusSignal<AngularVelocity> ShooterRPS = leaderMotor.getVelocity();
  private final StatusSignal<Voltage> ShooterAppliedVolts = leaderMotor.getMotorVoltage();
  private final StatusSignal<Current> ShooterCurrent = leaderMotor.getSupplyCurrent();

  // Declare hood status signals
  private final StatusSignal<Angle> HoodPosition = hoodMotor.getPosition();
  private final StatusSignal<Current> HoodCurrent = hoodMotor.getSupplyCurrent();

  // Declare turret status signals
  private final StatusSignal<Angle> TurretPosition = turretMotor.getPosition();
  private final StatusSignal<Current> TurretCurrent = turretMotor.getSupplyCurrent();

  // Declare input requests
  private VoltageOut shooterVoltageRequest = new VoltageOut(0);
  private MotionMagicVelocityVoltage shooterVelocityRequest = new MotionMagicVelocityVoltage(0);
  private PositionVoltage hoodPositionRequest = new PositionVoltage(0);
  private VoltageOut turretVoltageRequest = new VoltageOut(0);
  private MotionMagicVoltage turretPositionRequest = new MotionMagicVoltage(0);

  public ShooterIOTalonFX() {
    // Attempt to configure all motors.
    tryUntilOk(
        5, () -> leaderMotor.getConfigurator().apply(Robot.hardwareConfigs.shooterConfig, 0.25));
    tryUntilOk(
        5, () -> followerMotor.getConfigurator().apply(Robot.hardwareConfigs.shooterConfig, 0.25));
    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(Robot.hardwareConfigs.hoodConfig, 0.25));
    tryUntilOk(
        5, () -> turretMotor.getConfigurator().apply(Robot.hardwareConfigs.turretConfig, 0.25));

    // Update base status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        ShooterRPS,
        ShooterAppliedVolts,
        ShooterCurrent,
        HoodPosition,
        HoodCurrent,
        TurretPosition,
        TurretCurrent);
    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
    turretMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        ShooterRPS,
        ShooterAppliedVolts,
        ShooterCurrent,
        HoodPosition,
        HoodCurrent,
        TurretPosition,
        TurretCurrent);

    inputs.ShooterVelocityRPS = ShooterRPS.getValueAsDouble();
    inputs.ShooterVelocityMPS =
        ShooterRPS.getValueAsDouble()
            * Units.inchesToMeters(Constants.Shooter.shooterWheelDiameter * Math.PI);
    inputs.ShooterAppliedVolts = ShooterAppliedVolts.getValueAsDouble();
    inputs.ShooterAppliedAmps = ShooterCurrent.getValueAsDouble();

    inputs.HoodPosition = HoodPosition.getValueAsDouble();
    inputs.HoodCurrentAmps = HoodCurrent.getValueAsDouble();

    inputs.TurretPosition = TurretPosition.getValueAsDouble();
    inputs.TurretCurrentAmps = TurretCurrent.getValueAsDouble();
  }

  // Shooter functions
  @Override
  public void setShooterVoltage(double Volts) {
    leaderMotor.setControl(shooterVoltageRequest.withOutput(Volts));
  }

  @Override
  public void setShooterRPS(double RPS) {
    leaderMotor.setControl(shooterVelocityRequest.withVelocity(RPS));
  }

  @Override
  public void setShooterMPS(double MPS) {
    leaderMotor.setControl(
        shooterVelocityRequest.withVelocity(
            MPS / Units.inchesToMeters(Constants.Shooter.shooterWheelDiameter * Math.PI)));
  }

  // Hood functions
  @Override
  public void setShooterHoodAngle(double Degrees) {
    hoodMotor.setControl(hoodPositionRequest.withPosition(Degrees));
  }

  // Turret functions
  @Override
  public void setTurretVoltage(double Volts) {
    turretMotor.setControl(turretVoltageRequest.withOutput(Volts));
  }

  @Override
  public void setTurretAngle(double Degrees) {
    turretMotor.setControl(turretPositionRequest.withPosition(Degrees));
  }
}
