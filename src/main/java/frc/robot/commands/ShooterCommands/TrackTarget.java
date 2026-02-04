// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
public class TrackTarget extends Command {
  private final Shooter shooter;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final Supplier<Pose2d> targetSupplier;

  private Pose2d targetPose;

  /**
   * Primary constructor where both a robot-pose supplier and an explicit target supplier are
   * provided.
   */
  public TrackTarget(
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
  public TrackTarget(Shooter shooter, Supplier<Pose2d> robotPoseSupplier) {
    this(shooter, robotPoseSupplier, () -> shooter.getSmartTargetPose(robotPoseSupplier.get()));
  }

  /** Convenience constructor that uses the shooter-configured robot pose supplier. */
  public TrackTarget(Shooter shooter) {
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

    // Final shooter pose
    Pose2d shooterPose = new Pose2d(shooterPositionPose.getTranslation(), shooterHeading);

    double shooterSetpoint = shooter.calculateTurretAngleToGoal(shooterPose, targetPose);

    double driveHeading = robotPose.getRotation().getDegrees();
    double turretTarget = shooterSetpoint - driveHeading;

    turretTarget = MathUtil.inputModulus(turretTarget, 0, 358);
    shooter.setTurretAngle(turretTarget);

    // Hood ranging
    double shotDistanceMeters = shooter.calculateDistanceFromGoal(shooterPose, targetPose);

    double hoodSetpoint;
    if (shooter.isInTrench(shooterPose)) {
      hoodSetpoint = 0;
    } else {
      hoodSetpoint = shooter.getMappedHoodAngle(shotDistanceMeters);
    }

    shooter.setHoodAngle(hoodSetpoint);
    shooter.setShooterRPS(shooter.getMappedVelocity(shotDistanceMeters));

    Logger.recordOutput("Shooter/Angle setpoint", shooterSetpoint);
    Logger.recordOutput("Shooter/Pose", shooterPose);
    Logger.recordOutput("Shooter/Shot distance meters", shotDistanceMeters);
    Logger.recordOutput("Shooter/InAllianceZone", shooter.isInAllianceZone(robotPose));
    Logger.recordOutput("Shooter/isInTrench", shooter.isInTrench(shooterPose));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
