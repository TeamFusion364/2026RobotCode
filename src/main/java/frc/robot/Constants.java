// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class auto {

    // Closed-loop control constants
    public static final double autoDriveKp = 1.0;
    public static final double autoDriveKi = 0;
    public static final double autoDriveKd = 0;
    public static final double maxVelocityMPS = 3; // Meters per second
    public static final double driveTimeToAccel = 0.5;
    public static final double maxAccel = maxVelocityMPS / driveTimeToAccel;
    public static final double positionDeadband = 0.5; // Inches

    public static final double autoTurnKp = 2.0;
    public static final double autoTurnKi = 0;
    public static final double autoTurnKd = 0;
    public static final double autoTurnVelocityRPS = 2; // Radians per second
    public static final double turnTimeToAccel = 0.5;
    public static final double autoTurnMaxAccel = autoTurnVelocityRPS / turnTimeToAccel;
    public static final double angleDeadband = 2; // Degrees
  }

  public static final class Shooter {

    // CAN IDs
    public static final int leaderMotorID = 9;
    public static final int followerMotorID = 10;

    public static final int hoodMotorID = 11;

    public static final int turretMotorID = 12;

    // direction inverts
    public static final boolean invertLeader = false;
    public static final boolean invertFollower = true;

    public static final boolean invertHood = false;
    public static final boolean invertTurret = false;

    // Conversion ratios
    public static final double shooterGearRatio = 1 / 1;
    public static final double shooterWheelDiameter = 4;
    public static final double FlywheelWeightKG = 1.0;

    public static final double hoodGearRatio = (24 / 12) * (320 / 20);

    public static final double turretGearRatio = (32 / 12) * (85 / 10);

    public static final Transform2d shooterOffset =
        new Transform2d(new Translation2d(0.1525, -0.1525), new Rotation2d());

    // Closed-loop control constants
    // Shooter motors
    public static final double shooterKp = 0.1;
    public static final double shooterKi = 0;
    public static final double shooterKd = 0;
    public static final double shooterKv = 0.12;
    public static final double shooterKa = 0.18;
    public static final double shooterKs = 0.192;
    public static final double shooterMaxAccel = 90;
    public static final double shooterSpinupTime = 0.01;
    public static final double shooterMaxJerk = shooterMaxAccel / shooterSpinupTime;
    public static final CurrentLimitsConfigs shooterCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(75)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerTime(0.15);

    // Hood motor
    public static final double hoodForwardLimit = 20;
    public static final double hoodReverseLimit = 0;
    public static final double hoodDeadband = 0.25;
    public static final double hoodKp = 0.01;
    public static final double hoodKi = 0;
    public static final double hoodKd = 0;
    public static final CurrentLimitsConfigs hoodCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(15)
            .withSupplyCurrentLowerLimit(10)
            .withSupplyCurrentLowerTime(0.15);

    // Turret motor
    public static final double turretForwardLimit = 350;
    public static final double turretReverseLimit = 0;
    public static final double turretDeadband = 0.5;
    public static final double turretKp = 0.01;
    public static final double turretKi = 0;
    public static final double turretKd = 0;
    public static final double turretMaxVel = 200;
    public static final double turretAccelTime = 0.1;
    public static final double turretMaxAccel = turretMaxVel / turretAccelTime;
    public static final CurrentLimitsConfigs turretCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerTime(0.15);

    public static final double stationaryToleranceMetersPerSecond = 0.01;
    public static final double stationaryToleranceDegreesPerSecond = 2;
  }

  public static final class intake {

    // CAN IDs
    public static final int strokerID = 13;
    public static final int intakeID = 14;

    // direction inverts
    public static final boolean invertStroker = true;
    public static final boolean invertIntake = true;

    // Conversion ratios
    public static final double strokerGearRatio = 48 / 16;
    public static final double strokerGearDiameter = 1;

    public static final double strokerConversionRatio =
        (strokerGearDiameter * Math.PI) / strokerGearRatio;

    // Closed-loop control constants
    // Stroker motor
    public static final double strokerForwardLimit = 11.5; // Inches
    public static final double strokerReverseLimit = 0.0; // Inches
    public static final double strokerDeadband = 0.25;
    public static final double strokerKp = 0.65;
    public static final double strokerKi = 0;
    public static final double strokerKd = 0;
    public static final double strokerMaxVel = 24;
    public static final double strokerAccelTime = 0.1;
    public static final double strokerMaxAccel = strokerMaxVel / strokerAccelTime;
    public static final CurrentLimitsConfigs strokerCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLowerLimit(20)
            .withSupplyCurrentLowerTime(0.15);
  }

  public static final class feeder {

    // CAN IDs
    public static final int indexerID = 15;
    public static final int kickerID = 16;

    // direction inverts
    public static final boolean invertIndexer = false;
    public static final boolean invertKicker = true;

    public static final CurrentLimitsConfigs feederCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerTime(0.15);
  }

  public static final class Climber {

    // CAN IDs
    public static final int climberID = 17;

    // direction inverts
    public static final boolean invertClimber = false;

    // closed-loop control constants
    public static final double climberForwardLimit = 113.0; // rotations
    public static final double climberReverseLimit = 0.0; // rotations
    public static final double climberDeadband = 0.25;
    public static final double climberKp = 0.02;
    public static final double climberKi = 0;
    public static final double climberKd = 0;

    public static final CurrentLimitsConfigs climberCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerTime(0.15);
  }

  public static final class ShooterMaps {
    // TODO: These MUST be calibrated
    public static final double[][] HubMap = {
      // distance to goal, hood angle, linear shooter velocity
      {1.00, 0.0, 30},
      {2.00, 10.0, 40},
      {3.75, 15.0, 50},
      {5.25, 20.0, 60}
    };

    public static final double[][] TOFMap = {
      // distance to goal, Time of flight
      {1.00, 0.35},
      {2.00, 0.5},
      {3.00, 0.75},
      {5.0, 0.9},
      {10.0, 1.1},
      {15.0, 1.2}
    };
  }
}
