// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Shooter;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Generic command that tracks a target pose supplied by a Pose2d Supplier (robot pose supplier).
 * The command will use the provided robotPose supplier to derive the target pose via {@link
 * Shooter#getSmartTargetPose(Pose2d)} unless an explicit target supplier is provided.
 */
public class TrackTargetLive extends Command {
  private final Shooter shooter;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final Supplier<Pose2d> targetSupplier;

  private Pose2d targetPose;

  private Pose2d AdjustedTargetPose;

  /**
   * Primary constructor where both a robot-pose supplier and an explicit target supplier are
   * provided.
   */
  public TrackTargetLive(
      Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> targetSupplier) {
    this.shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetSupplier = targetSupplier;

    addRequirements(shooter);
  }

  /**
   * Constructor that accepts only a robot-pose supplier. The target pose will be computed by the
   * shooter using getSmartTargetPose(robotPose).
   */
  public TrackTargetLive(Shooter shooter, Supplier<Pose2d> robotPoseSupplier) {
    this(shooter, robotPoseSupplier, () -> shooter.getSmartTargetPose(robotPoseSupplier.get()));
  }

  /** Convenience constructor that uses the shooter-configured robot pose supplier. */
  public TrackTargetLive(Shooter shooter) {
    this(shooter, shooter::getRobotPose);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Retrieve robot pose and target from suppliers
    Pose2d robotPose = robotPoseSupplier.get();
    targetPose = targetSupplier.get();

    // Turret aiming (with shooter offset)
    Pose2d shooterPositionPose = robotPose.plus(Constants.Shooter.shooterOffset);

    // Shooter heading = robot heading + turret angle
    Rotation2d shooterHeading =
        robotPose.getRotation().plus(Rotation2d.fromDegrees(shooter.getTurretAngle()));

    // nominal shooter pose
    Pose2d shooterPose = new Pose2d(shooterPositionPose.getTranslation(), shooterHeading);

    double shooterSetpoint = shooter.calculateTurretAngleToGoal(shooterPose, targetPose);

    double driveHeading = robotPose.getRotation().getDegrees();

    double shotDistanceMeters = shooter.calculateDistanceFromGoal(shooterPose, targetPose);

    // Calculate field relative turret velocity
    Translation2d robotToTurret = Constants.Shooter.shooterOffset.getTranslation();
    ChassisSpeeds robotVelocity =
        shooter
            .getChassisSpeed()
            .fromRobotRelativeSpeeds(shooter.getChassisSpeed(), robotPose.getRotation());
    double robotAngle = robotPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Calculate look-ahead value for shoot on the move
    double timeOfFlight = shooter.getMappedTOF(shotDistanceMeters);
    Pose2d lookAheadPose;
    double TurretToTargetDistance;

    double offsetX = turretVelocityX * timeOfFlight;
    double offsetY = turretVelocityY * timeOfFlight;
    lookAheadPose =
        new Pose2d(
            targetPose.getTranslation().minus(new Translation2d(offsetX, offsetY)),
            targetPose.getRotation());

    TurretToTargetDistance =
        targetPose.getTranslation().getDistance(lookAheadPose.getTranslation());

    double turretTarget = shooter.calculateTurretAngleToGoal(shooterPose, lookAheadPose);

    double adjustedShotDistanceMeters =
        shooter.calculateDistanceFromGoal(shooterPose, lookAheadPose);

    // Hood ranging
    double hoodSetpoint;
    if (shooter.isInTrench(shooterPose)) {
      hoodSetpoint = 0;
    } else {
      hoodSetpoint = shooter.getMappedHoodAngle(adjustedShotDistanceMeters);
    }

    turretTarget = MathUtil.inputModulus(turretTarget - driveHeading, 0, 358);
    shooter.setTurretAngle(turretTarget);

    shooter.setHoodAngle(hoodSetpoint);
    shooter.setShooterRPS(shooter.getMappedVelocity(adjustedShotDistanceMeters));

    Logger.recordOutput("Shooter/Angle setpoint", shooterSetpoint);
    Logger.recordOutput("Shooter/Pose", shooterPose);
    Logger.recordOutput("Shooter/Target", lookAheadPose);
    Logger.recordOutput("Shooter/Hood setpoint", hoodSetpoint);
    Logger.recordOutput("Shooter/Shot distance meters", adjustedShotDistanceMeters);
    Logger.recordOutput("Shooter/InAllianceZone", shooter.isInAllianceZone(robotPose));
    Logger.recordOutput("Shooter/isInTrench", shooter.isInTrench(shooterPose));
    Logger.recordOutput("Shooter/TOF", shooter.getMappedTOF(adjustedShotDistanceMeters));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
