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
public class setIntakeVoltage extends Command {
  private final Intake intake;
  private final double voltage;

  /**
   * Create a new SetIntakePosition command.
   *
   * @param intake the intake subsystem\
   * @param voltage the voltage to set the intake to
   */
  public setIntakeVoltage(Intake intake, double voltage) {
    this.intake = intake;
    this.voltage = voltage;

    addRequirements();
  }

  @Override
  public void initialize() {
    intake.setIntakeVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // Command ends immediately after setting the position
    return true;
  }
}
