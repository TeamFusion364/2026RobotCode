// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import frc.robot.subsystems.Shooter.Shooter;

/**
 * Backwards-compatible wrapper for the old SmartTrack name. Delegates to the new TrackTarget
 * implementation using the shooter's configured robot-pose supplier.
 */
public class SmartTrack extends TrackTarget {
  public SmartTrack(Shooter shooter) {
    super(shooter);
  }
}
