// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackHub extends Command {
  /** Creates a new hub tracking command for aiming and ranging */
  private Shooter shooter;

  private Drive drive;

  private Pose2d targetPose;

  public TrackHub(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // flip the hub target pose based on alliance color
    if (shooter.getShooterFlipped() == true) {
      targetPose = FieldConstants.RedHubCenter;
    } else {
      targetPose = FieldConstants.BlueHubCenter;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turret aiming
    Pose2d shooterPose =
        new Pose2d(
            drive.getPose().getX(),
            drive.getPose().getY(),
            new Rotation2d(
                Units.degreesToRadians(
                    shooter.getTurretAngle() + drive.getPose().getRotation().getDegrees())));
    double shooterSetpoint = shooter.calculateTurretAngleToGoal(shooterPose, targetPose);
    // double flipMulti = shooter.getShooterFlipped() == true ? -1 : 1;
    double driveHeading = drive.getPose().getRotation().getDegrees();
    double turretCurrentAngle = shooter.getTurretAngle();
    double turretTarget = shooterSetpoint - driveHeading;

    turretTarget = MathUtil.inputModulus(turretTarget, 0, 358);
    shooter.setTurretAngle(turretTarget);

    // Hood ranging
    // Distance from turret to hub center
    double shotDistanceMeters = shooter.calculateDistanceFromGoal(shooterPose, targetPose);

    // Apply values interpolated from distance to hood angle/shooter wheel velocity
    shooter.setHoodAngle(shooter.getMappedHoodAngle(shotDistanceMeters));
    shooter.setShooterRPS(shooter.getMappedVelocity(shotDistanceMeters));

    Logger.recordOutput("Shooter/Angle setpoint", shooterSetpoint);
    Logger.recordOutput("Shooter/Pose", shooterPose);
    Logger.recordOutput("Shooter/Shot distance meters", shotDistanceMeters);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTurretAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
