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
 * Command that tracks the goal with the turret only, while keeping the flywheel at 3 RPS and the
 * hood retracted (0 degrees). The command will use the provided robotPose supplier to derive the
 * target pose via {@link Shooter#getSmartTargetPose(Pose2d)}.
 */
public class TrackGoalOnly extends Command {
  private final Shooter shooter;

  private final Supplier<Pose2d> robotPoseSupplier;

  private Pose2d targetPose;

  /**
   * Constructor that accepts a robot-pose supplier. The target pose will be computed by the shooter
   * using getSmartTargetPose(robotPose).
   */
  public TrackGoalOnly(Shooter shooter, Supplier<Pose2d> robotPoseSupplier) {
    this.shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;

    addRequirements(shooter);
  }

  /** Convenience constructor that uses the shooter-configured robot pose supplier. */
  public TrackGoalOnly(Shooter shooter) {
    this(shooter, shooter::getRobotPose);
  }

  @Override
  public void initialize() {
    // Keep flywheel at 3 RPS and hood retracted
    shooter.setShooterRPS(3.0);
    shooter.setHoodAngle(0.0);
  }

  @Override
  public void execute() {
    // Retrieve robot pose and target from suppliers
    Pose2d robotPose = robotPoseSupplier.get();
    targetPose = shooter.getSmartTargetPose(robotPose);

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

    // Keep flywheel at 3 RPS and hood retracted
    shooter.setShooterRPS(3.0);
    shooter.setHoodAngle(0.0);

    Logger.recordOutput("Shooter/Angle setpoint", shooterSetpoint);
    Logger.recordOutput("Shooter/Pose", shooterPose);
    Logger.recordOutput("Shooter/Target", targetPose);
    Logger.recordOutput("Shooter/Hood setpoint", 0.0);
    Logger.recordOutput(
        "Shooter/TOF",
        shooter.getMappedTOF(shooter.calculateDistanceFromGoal(shooterPose, targetPose)));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
