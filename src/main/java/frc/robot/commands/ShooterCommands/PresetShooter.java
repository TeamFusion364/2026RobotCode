// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Sets the turret angle, hood angle, and shooter velocity to preset values */
public class PresetShooter extends Command {
  private final Shooter shooter;

  double hoodSetpoint;
  double shooterSetpoint;
  double velocitySetpoint;

  private Pose2d targetPose;

  /**
   * Primary constructor where both a robot-pose supplier and an explicit target supplier are
   * provided.
   */
  public PresetShooter(
      Shooter shooter,
      DoubleSupplier turretAngleSupplier,
      DoubleSupplier hoodAngleSupplier,
      DoubleSupplier velocitySupplier) {
    this.shooter = shooter;

    hoodSetpoint = hoodAngleSupplier.getAsDouble();
    shooterSetpoint = turretAngleSupplier.getAsDouble();
    velocitySetpoint = velocitySupplier.getAsDouble();

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    shooter.setTurretAngle((shooterSetpoint - 90));
    shooter.setHoodAngle(SmartDashboard.getNumber("SM-Hood", 0));
    shooter.setShooterRPS(SmartDashboard.getNumber("SM-RPS", 40));

    Logger.recordOutput("Shooter/Angle setpoint", shooterSetpoint);
    Logger.recordOutput("Shooter/Hood setpoint", hoodSetpoint);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
