// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PulseIntake extends SequentialCommandGroup {

  private Intake intake;
  public double RT = 0.75; // Time spent retracting in seconds
  public double ET = 0.5; // Time spend extending in seconds
  /** Creates a new PulseIntake. */
  public PulseIntake(Intake intake) {
    this.intake = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> intake.setIntakeVoltage(-8)),
        new InstantCommand(() -> intake.setStrokerPositionInches(7.5)),
        new WaitCommand(ET),
        new InstantCommand(() -> intake.setStrokerPositionInches(10.75)),
        new WaitCommand(RT),
        new InstantCommand(() -> intake.setStrokerPositionInches(5)),
        new WaitCommand(ET),
        new InstantCommand(() -> intake.setStrokerPositionInches(10.75)),
        new WaitCommand(RT),
        new InstantCommand(() -> intake.setStrokerPositionInches(2.5)),
        new WaitCommand(ET),
        new InstantCommand(() -> intake.setIntakeVoltage(0)),
        new InstantCommand(() -> intake.setStrokerPositionInches(11)));
  }
}
