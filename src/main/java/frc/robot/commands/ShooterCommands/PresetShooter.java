// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Sets the turret angle, hood angle, and shooter velocity to preset values
 */
public class PresetShooter extends Command {
  private final Shooter shooter;

  private final DoubleSupplier turretAngleSupplier;
  private final DoubleSupplier hoodAngleSupplier;
  private final DoubleSupplier velocitySupplier;

  double hoodSetpoint;
  double shooterSetpoint;
  double velocitySetpoint;

  private Pose2d targetPose;

  /**
   * Primary constructor where both a robot-pose supplier and an explicit target supplier are
   * provided.
   */
  public PresetShooter(
      Shooter shooter, DoubleSupplier turretAngleSupplier, DoubleSupplier hoodAngleSupplier, DoubleSupplier velocitySupplier) {
    this.shooter = shooter;
    this.turretAngleSupplier = turretAngleSupplier;
    this.hoodAngleSupplier = hoodAngleSupplier;
    this.velocitySupplier = velocitySupplier;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {

    double hoodSetpoint = hoodAngleSupplier.getAsDouble();
    double shooterSetpoint = turretAngleSupplier.getAsDouble();
    double velocitySetpoint = velocitySupplier.getAsDouble();

  }

  @Override
  public void execute() {
    
    shooter.setHoodAngle(hoodSetpoint);
    shooter.setShooterRPS(velocitySetpoint);

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
