// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Kicker.Kicker;

/** Simple command to set the intake stroker to a desired position (inches). */
public class UnjamIntake extends Command {
  private final Intake intake;
  private final Feeder feeder;
  private final Kicker kicker;

  /**
   * Create a new SetIntakePosition command.
   *
   * @param intake the intake subsystem
   */
  public UnjamIntake(Intake intake, Feeder feeder, Kicker kicker) {
    this.intake = intake;
    this.feeder = feeder;
    this.kicker = kicker;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setStrokerPositionInches(11.3);
    intake.setIntakeVoltage(6);
    feeder.setFeederVoltage(4);
    kicker.setKickerVoltage(4);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // Command ends immediately after setting the position
    return intake.isStrokerAtSetpoint(11.3);
    // return false;
  }
}
