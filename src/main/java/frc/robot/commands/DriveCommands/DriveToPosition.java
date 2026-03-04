// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/* Create a command to drive to a pose using a profiled PID controller */
public class DriveToPosition extends Command {

  public Drive drive;
  public final Pose2d targetPose;
  public PPHolonomicDriveController driveController;

  private final Trigger endTrigger;
  private final Trigger endTriggerDebounced;

  /** Creates a new DriveToPosition command */
  private DriveToPosition(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.targetPose = targetPoseSupplier.get();

    driveController =
        new PPHolonomicDriveController(
            new PIDConstants(
                Constants.auto.autoDriveKp, Constants.auto.autoDriveKi, Constants.auto.autoDriveKd),
            new PIDConstants(
                Constants.auto.autoTurnKp, Constants.auto.autoTurnKi, Constants.auto.autoTurnKd));
    ;

    endTrigger =
        new Trigger(
            () -> {
              boolean x =
                  MathUtil.isNear(
                      targetPose.getX(),
                      drive.getPose().getX(),
                      Units.inchesToMeters(Constants.auto.positionDeadband));
              boolean y =
                  MathUtil.isNear(
                      targetPose.getY(),
                      drive.getPose().getY(),
                      Units.inchesToMeters(Constants.auto.positionDeadband));
              boolean rotation =
                  MathUtil.isNear(
                      targetPose.getRotation().getDegrees(),
                      drive.getPose().getRotation().getDegrees(),
                      Constants.auto.angleDeadband);
              boolean speed = (Math.abs(drive.getVelocityMPS()) < 0.1);
              return x && y && rotation && speed;
            });
    endTriggerDebounced = endTrigger.debounce(0.1);
  }

  public static Command generateCommand(Drive drive, Pose2d targetPose, double timeout) {
    // Handle flipping for alliance color
    Command noFlip = new DriveToPosition(drive, () -> targetPose).withTimeout(timeout);
    Command flip =
        new DriveToPosition(drive, () -> FlippingUtil.flipFieldPose(targetPose))
            .withTimeout(timeout);

    return new ConditionalCommand(flip, noFlip, () -> drive.getFlipped());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
    targetState.pose = targetPose;
    drive.runVelocity(driveController.calculateRobotRelativeSpeeds(drive.getPose(), targetState));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endTriggerDebounced.getAsBoolean();
  }
}
