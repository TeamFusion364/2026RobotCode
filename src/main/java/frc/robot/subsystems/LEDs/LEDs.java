// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem that controls an addressable LED strip using a CANdle. */
public class LEDs extends SubsystemBase {
  private final CANdle candle;
  private final CANdleConfiguration candleConfig;

  public LEDs() {
    candle = new CANdle(Constants.LEDs.CANdleID, CANBus.roboRIO());
    candleConfig =
        new CANdleConfiguration()
            .withLED(
                new LEDConfigs()
                    .withStripType(StripTypeValue.GRB)
                    .withBrightnessScalar(0.5)
                    .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs));

    candle.getConfigurator().apply(candleConfig);
    setOrangeFade();
  }

  public void setWhiteStrobe() {
    candle.setControl(
        new StrobeAnimation(0, Constants.LEDs.LEDLength)
            .withColor(new RGBWColor(0, 0, 0, 255))
            .withFrameRate(Hertz.of(200)));
  }

  public void setBlueWave() {
    candle.setControl(
        new LarsonAnimation(0, Constants.LEDs.LEDLength)
            .withColor(new RGBWColor(0, 0, 255, 0))
            .withFrameRate(Hertz.of(200)));
  }

  public void setOrangeFade() {
    candle.setControl(
        new SingleFadeAnimation(0, Constants.LEDs.LEDLength)
            .withColor(new RGBWColor(150, 100, 0, 0))
            .withFrameRate(Hertz.of(100)));
  }
}
