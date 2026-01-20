// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Store field position values */
public class FieldConstants {
  // All field positions are for the blue side. Use flipping util to change to red alliance
  // positions

  // Center of the hub location
  public static final Pose2d HubCenter = new Pose2d(4.625, 4.050, new Rotation2d(0));
}
