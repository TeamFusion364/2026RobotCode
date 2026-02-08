// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

/** Simple command to set the intake stroker to a desired position (inches). */
public class SetIntakePosition extends Command {
  private final Intake intake;
  private final double positionInches;

  /**
   * Create a new SetIntakePosition command.
   *
   * @param intake the intake subsystem
   * @param positionInches the target position in inches
   */
  public SetIntakePosition(Intake intake, double positionInches) {
    this.intake = intake;
    this.positionInches = positionInches;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setStrokerPositionInches(positionInches);
    intake.setIntakeVoltage(8);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // Command ends immediately after setting the position
    return intake.isStrokerAtSetpoint();
  }
}
