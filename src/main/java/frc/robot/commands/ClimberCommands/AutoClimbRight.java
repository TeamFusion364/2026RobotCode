// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveCommands.DriveToPosition;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimbRight extends SequentialCommandGroup {
  /** Creates a new AutoClimb. */
  public AutoClimbRight(Drive drive, Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        DriveToPosition.generateCommand(drive, FieldConstants.RightClimbPrime, 10),
        new SetClimberPosition(climber, () -> 112),
        DriveToPosition.generateCommand(drive, FieldConstants.RightClimbHook, 10),
        new SetClimberPosition(climber, () -> 50));
  }
}
