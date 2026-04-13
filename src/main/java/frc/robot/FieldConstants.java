// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Store field position values */
public class FieldConstants {

  // Center of the hub location
  public static final Pose2d BlueHubCenter = new Pose2d(4.625, 4.050, new Rotation2d(0));
  public static final Pose2d RedHubCenter = FlippingUtil.flipFieldPose(BlueHubCenter);

  // Feeding location (Right)
  // public static final Pose2d RedFeedingTarget = new Pose2d(1.25, 1.00, new Rotation2d());
  // public static final Pose2d BlueFeedingTarget = FlippingUtil.flipFieldPose(RedFeedingTarget);

  // Feeding location (Left)
  public static final Pose2d BlueFeedingTarget = new Pose2d(1.93, 1.8, new Rotation2d());
  public static final Pose2d RedFeedingTarget = new Pose2d(1.93, 1.8, new Rotation2d());

  // Feeding location (Right)
  public static final Pose2d BlueRightFeedingTarget = new Pose2d(1.93, 1.8, new Rotation2d());
  public static final Pose2d RedRightFeedingTarget = FlippingUtil.flipFieldPose(BlueFeedingTarget);

  // Climb poses
  public static final Pose2d LeftClimbPrime = new Pose2d();
  public static final Pose2d LeftClimbHook = new Pose2d();

  public static final Pose2d RightClimbPrime =
      new Pose2d(1.500, 3.14, new Rotation2d(Units.degreesToRadians(180))); // red
  public static final Pose2d RightClimbHook =
      new Pose2d(1.500, 3.312, new Rotation2d(Units.degreesToRadians(180))); // red
}
