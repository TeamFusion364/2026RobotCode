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

/** Simple command to set the climber to a desired voltage (0-12). */
public class SetClimberVoltage extends Command {

  private final Climber climber;
  private final double voltage;

  /**
   * Create a new SetClimberVoltage command.
   *
   * @param climber the climber subsystem
   */
  public SetClimberVoltage(Climber climber, DoubleSupplier voltageSupplier) {
    this.climber = climber;
    this.voltage = voltageSupplier.getAsDouble();

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setClimberVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // Command ends immediately after setting the voltage
    return false;
  }
}
