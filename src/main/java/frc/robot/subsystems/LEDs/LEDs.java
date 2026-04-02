// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem that controls an addressable LED strip using a CANdle. */
public class LEDs extends SubsystemBase {
  private final CANBus kCANBus = new CANBus("rio");
  private final CANdle m_candle = new CANdle(16, kCANBus);

  private final SolidColor[] m_colors =
      new SolidColor[] {
        new SolidColor(0, 68).withColor(RGBWColor.fromHSV(20, 80, 80)),
      };

  public LEDs() {
    setDefaultCommand(updateLEDs());
  }

  /**
   * Updates the animations and LEDs of the CANdle.
   *
   * @return Command to run
   */
  public Command updateLEDs() {
    return run(
        () -> {
          for (var solidColor : m_colors) {
            m_candle.setControl(solidColor);
          }
        });
  }
}
