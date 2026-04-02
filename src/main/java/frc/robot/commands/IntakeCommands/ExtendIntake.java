// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

/** Simple command to set the intake stroker to a desired position (inches). */
public class ExtendIntake extends Command {
  private final Intake intake;

  /**
   * Create a new SetIntakePosition command.
   *
   * @param intake the intake subsystem
   */
  public ExtendIntake(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setStrokerPositionInches(11.3);
  }

  @Override
  public void execute() {
    intake.setIntakeVoltage(6);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeVoltage(0);
  }

  @Override
  public boolean isFinished() {
    // Command ends immediately after setting the position
    return intake.isStrokerAtSetpoint(11.3);
    // return false;
  }
}
