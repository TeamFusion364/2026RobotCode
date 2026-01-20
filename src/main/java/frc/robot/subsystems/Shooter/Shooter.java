// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Creates a new turret subsystem. Advantagekit/sim compatible */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;

    // Initiation code goes here
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  // Set shooter raw voltage
  public void setShooterVoltage(double Volts) {
    io.setShooterVoltage(Volts);
  }

  // Set shooter velocity in RPS
  public void setShooterRPS(double RPS) {
    io.setShooterRPS(RPS);
  }

  // Set shooter velocity in MPS
  public void setShooterMPS(double MPS) {
    io.setShooterMPS(MPS);
  }

  // Set hood angle in Degrees
  public void setHoodAngle(double Degrees) {
    io.setShooterHoodAngle(Degrees);
  }

  // Set turret angle in Degrees
  public void setTurretAngle(double Degrees) {
    io.setTurretAngle(Degrees);
  }

  // Set turret raw voltage
  public void setTurretVoltage(double Volts) {
    io.setTurretVoltage(Volts);
  }
}
