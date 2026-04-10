// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Kicker.Kicker;

/** Set indexer subsystems to feed the shooter. */
public class FeedShooter extends Command {
  private final Feeder feeder;
  private final Kicker kicker;

  /** Create a new FeedShooter command. */
  public FeedShooter(Feeder feeder, Kicker kicker) {
    this.feeder = feeder;
    this.kicker = kicker;

    addRequirements(feeder, kicker);
  }

  @Override
  public void initialize() {
    feeder.setFeederVoltage(12);
    kicker.setKickerVoltage(-11);
  }

  @Override
  public void end(boolean interrupted) {
    feeder.setFeederVoltage(0);
    kicker.setKickerVoltage(0);
  }

  @Override
  public boolean isFinished() {
    // Command runs until interrupted
    return false;
  }
}
