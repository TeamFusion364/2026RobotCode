// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

/** Creates ShooterIO for Advantagekit */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // Shooter
    public double ShooterVelocityRPS = 0.0;
    public double ShooterVelocityMPS = 0.0;
    public double ShooterAppliedVolts = 0.0;
    public double ShooterAppliedAmps = 0.0;

    // Hood
    public double HoodPosition = 0.0;
    public double HoodCurrentAmps = 0.0;

    // Turret
    public double TurretPosition = 0.0;
    public double TurretCurrentAmps = 0.0;
    ;
  }

  // Updates loggable inputs
  public default void updateInputs(ShooterIOInputs inputs) {}

  // Shooter functions
  // Set shooter wheel using voltage (-12 - 12)
  public default void setShooterVoltage(double Volts) {}
  // Set shooter wheel speed using velocity PID (Rotations per second)
  public default void setShooterRPS(double RPS) {}
  // Set shooter linear projectile velocity
  public default void setShooterMPS(double MPS) {}

  // Hood functions
  // Set hood angle using position PID (Degrees)
  public default void setShooterHoodAngle(double Degrees) {}

  // Turret functions
  // Set turret rotation using voltage (-12 - 12)
  public default void setTurretVoltage(double Volts) {}
  // Set turret angle using profiled PID (degrees)
  public default void setTurretAngle(double Degrees) {}
}
