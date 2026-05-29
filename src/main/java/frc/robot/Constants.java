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
    public static final boolean invertLeader = true;
    public static final boolean invertFollower = false;

    public static final boolean invertHood = false;
    public static final boolean invertTurret = true;

    // Conversion ratios
    public static final double shooterGearRatio = 1 / 1;
    public static final double shooterWheelDiameter = 4;
    public static final double FlywheelWeightKG = 1.0;

    public static final double hoodGearRatio = 1 / 1;

    public static final double turretGearRatio = 22.71;

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
    public static final double hoodForwardLimit = 3;
    public static final double hoodReverseLimit = 0;
    public static final double hoodDeadband = 0.09;
    public static final double hoodKp = 13;
    public static final double hoodKi = 0;
    public static final double hoodKd = 0;
    public static final CurrentLimitsConfigs hoodCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLowerLimit(430)
            .withSupplyCurrentLowerTime(0.15);

    // Turret motor
    public static final double turretForwardLimit = 0.05;
    public static final double turretReverseLimit = -1;
    public static final double turretDeadband = 1; // Degrees
    public static final double turretKp = 40;
    public static final double turretKi = 0;
    public static final double turretKd = 0;
    public static final double turretKs = 0.32;
    public static final double turretMaxVel = 3;
    public static final double turretAccelTime = 0.3;
    public static final double turretMaxAccel = turretMaxVel / turretAccelTime;
    public static final CurrentLimitsConfigs turretCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentLowerLimit(20)
            .withSupplyCurrentLowerTime(0.15);

    public static final double stationaryToleranceMetersPerSecond = 0.01;
    public static final double stationaryToleranceDegreesPerSecond = 2;
  }

  public static final class intake {

    // CAN IDs
    public static final int strokerID = 13;
    public static final int intakeID = 14;
    public static final int intakeFollowerID = 44;

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
    public static final double strokerKp = 0.8;
    public static final double strokerKi = 0;
    public static final double strokerKd = 0;
    public static final double strokerMaxVel = 24;
    public static final double strokerAccelTime = 0.1;
    public static final double strokerMaxAccel = strokerMaxVel / strokerAccelTime;
    public static final CurrentLimitsConfigs strokerCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(12)
            .withSupplyCurrentLowerLimit(15)
            .withSupplyCurrentLowerTime(0.15);

    public static final CurrentLimitsConfigs intakeCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLowerLimit(30)
            .withSupplyCurrentLowerTime(0.15);
  }

  public static final class feeder {

    // CAN IDs
    public static final int indexerID = 15;
    public static final int kickerID = 16;
    public static final int indexerFollowerID = 17;

    // direction inverts
    public static final boolean invertIndexer = false;
    public static final boolean invertKicker = true;

    public static final CurrentLimitsConfigs feederCurrent =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLowerLimit(30)
            .withSupplyCurrentLowerTime(0.15);

    // Jam detection
    public static final double jamCurrentThreshold = 80; // Amps
    public static final double jamDetectionTimeThreshold = 10; // Seconds

    // Unjam logic
    public static final boolean enableAutomaticUnjam =
        false; // Enable/disable automatic unjam on jam detection
    public static final double unjamDuration = 0.5; // Seconds to run unjam when jam is detected
  }

  public static final class kicker {

    // Jam detection
    public static final double jamCurrentThreshold = 25.0; // Amps
    public static final double jamDetectionTimeThreshold = 0.5; // Seconds
  }

  // LED constants
  public static final class LEDs {

    // CANdle ID
    public static final int CANdleID = 16;

    // Amount of LEDs in the strip
    public static final int LEDLength = 68;
  }

  public static final class ShooterMaps {
    // TODO: These MUST be calibrated
    public static final double[][] HubMap = {
      // distance to goal, hood angle, linear shooter velocity
      {1, 0.0, 45},
      {2.00, 0.75, 48},
      {2.5, 1.00, 49},
      {3, 1.20, 50},
      {4, 1.45, 57.5},
      {4.3, 1.55, 59},
      {4.5, 1.6, 60},
      {5, 1.65, 62},
      {8, 1.75, 75}
    };

    public static final double[][] TOFMap = {
      // distance to goal, Time of flight
      {1.00, 0.9},
      {2.00, 1},
      {3.00, 1.1},
      {5.0, 1.2},
      {10.0, 1.15},
      {15.0, 1.25}
    };
  }
}
