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

/** Simple command to set the climber to a desired voltage until it reaches the desired setpoint */
public class SetClimberBangbang extends Command {

  private final Climber climber;
  private final double voltage;
  private final double setpoint;

  /**
   * Create a new SetClimberBangbang command.
   *
   * @param climber the climber subsystem
   */
  public SetClimberBangbang(
      Climber climber, DoubleSupplier voltageSupplier, DoubleSupplier setpointSupplier) {
    this.climber = climber;
    this.voltage = voltageSupplier.getAsDouble();
    this.setpoint = setpointSupplier.getAsDouble();

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setClimberVoltage(voltage);
  }

  @Override
  public void execute() {
    climber.setClimberVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setClimberVoltage(0);
  }

  @Override
  public boolean isFinished() {
    // Command ends immediately after reaching the setpoint
    return climber.isClimberAtSetpoint(setpoint);
    //
  }
}
