// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public final class HardwareConfigs {

  // TalonFX Configurations for hardware devices
  public TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
  public TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  public TalonFXConfiguration turretConfig = new TalonFXConfiguration();

  public TalonFXConfiguration strokerConfig = new TalonFXConfiguration();
  public TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  public TalonFXConfiguration feederConfig = new TalonFXConfiguration();

  public TalonFXConfiguration climberConfig = new TalonFXConfiguration();

  public HardwareConfigs() {

    // Shooter configuration
    // Mechanical configs
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.shooterGearRatio;
    shooterConfig.CurrentLimits = Constants.Shooter.shooterCurrent;

    // closed loop configs
    shooterConfig.Slot0.kP = Constants.Shooter.shooterKp;
    shooterConfig.Slot0.kI = Constants.Shooter.shooterKi;
    shooterConfig.Slot0.kD = Constants.Shooter.shooterKd;
    shooterConfig.Slot0.kV = Constants.Shooter.shooterKv;
    shooterConfig.Slot0.kA = Constants.Shooter.shooterKa;
    shooterConfig.Slot0.kS = Constants.Shooter.shooterKs;
    shooterConfig.MotionMagic.MotionMagicAcceleration = Constants.Shooter.shooterMaxAccel;
    shooterConfig.MotionMagic.MotionMagicJerk = Constants.Shooter.shooterMaxJerk;

    shooterConfig.MotorOutput.PeakForwardDutyCycle = 1;
    shooterConfig.MotorOutput.PeakReverseDutyCycle = 0;

    // Hood configuration
    // Mechanical configs
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.hoodGearRatio;
    hoodConfig.CurrentLimits = Constants.Shooter.hoodCurrent;

    // closed loop configs
    hoodConfig.Slot0.kP = Constants.Shooter.hoodKp;
    hoodConfig.Slot0.kI = Constants.Shooter.hoodKi;
    hoodConfig.Slot0.kD = Constants.Shooter.hoodKd;

    // Turret configuration
    // Mechanical configs
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.turretGearRatio;
    turretConfig.CurrentLimits = Constants.Shooter.turretCurrent;
    turretConfig.withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Constants.Shooter.turretForwardLimit)
            .withReverseSoftLimitThreshold(Constants.Shooter.turretReverseLimit));

    // closed loop configs
    turretConfig.Slot0.kP = Constants.Shooter.turretKp;
    turretConfig.Slot0.kI = Constants.Shooter.turretKi;
    turretConfig.Slot0.kD = Constants.Shooter.turretKd;
    turretConfig.Slot0.kS = Constants.Shooter.turretKs;
    turretConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.turretMaxVel;
    turretConfig.MotionMagic.MotionMagicAcceleration = Constants.Shooter.turretMaxAccel;

    // Stroker configuration
    // Mechanical configs
    strokerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    strokerConfig.Feedback.SensorToMechanismRatio = Constants.intake.strokerConversionRatio;
    strokerConfig.CurrentLimits = Constants.intake.strokerCurrent;

    // closed loop configs
    strokerConfig.Slot0.kP = Constants.intake.strokerKp;
    strokerConfig.Slot0.kI = Constants.intake.strokerKi;
    strokerConfig.Slot0.kD = Constants.intake.strokerKd;
    strokerConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.intake.strokerMaxVel;
    strokerConfig.MotionMagic.MotionMagicAcceleration = Constants.intake.strokerMaxAccel;

    // feeder configs
    // Mechanical configs
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.CurrentLimits = Constants.feeder.feederCurrent;

    // Intake config
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.CurrentLimits = Constants.intake.intakeCurrent;
  }
}
