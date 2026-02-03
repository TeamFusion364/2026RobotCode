// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Store field position values */
public class FieldConstants {

  // Center of the hub location
  public static final Pose2d BlueHubCenter = new Pose2d(4.625, 4.050, new Rotation2d(0));
  public static final Pose2d RedHubCenter = FlippingUtil.flipFieldPose(BlueHubCenter);

  // Feeding location
  public static final Pose2d RedFeedingTarget = new Pose2d(1.25, 1.00, new Rotation2d());
  public static final Pose2d BlueFeedingTarget = FlippingUtil.flipFieldPose(RedFeedingTarget);
}
