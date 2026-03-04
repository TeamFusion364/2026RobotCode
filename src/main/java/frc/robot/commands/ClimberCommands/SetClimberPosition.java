// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;
import java.util.function.DoubleSupplier;

/** Simple command to set the climber to a desired position (0-113). */
public class SetClimberPosition extends Command {

  private final Climber climber;
  private final double setpoint;

  /**
   * Create a new SetClimberPosition command.
   *
   * @param climber the climber subsystem
   */
  public SetClimberPosition(Climber climber, DoubleSupplier setpointSupplier) {
    this.climber = climber;
    this.setpoint = setpointSupplier.getAsDouble();

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setClimberPosition(setpoint);
  }

  @Override
  public void execute() {
    climber.setClimberPosition(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setClimberVoltage(0);
  }

  @Override
  public boolean isFinished() {
    // Command ends immediately after setting the position
    return climber.isClimberAtSetpoint(setpoint);
    // return false;
  }
}
