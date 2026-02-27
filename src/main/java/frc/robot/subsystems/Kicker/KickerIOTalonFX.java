package frc.robot.subsystems.Kicker;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Robot;

/** TalonFX-based Feeder IO implementation */
public class KickerIOTalonFX implements KickerIO {
  private final TalonFX KickerMotor = new TalonFX(Constants.feeder.kickerID);

  private final StatusSignal<Voltage> kickerVolt = KickerMotor.getMotorVoltage();
  private final StatusSignal<Current> kickerCurrent = KickerMotor.getSupplyCurrent();

  private VoltageOut kickerVoltageRequest = new VoltageOut(0);

  public KickerIOTalonFX() {
    tryUntilOk(
        5, () -> KickerMotor.getConfigurator().apply(Robot.hardwareConfigs.feederConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, kickerVolt, kickerCurrent);
    KickerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    BaseStatusSignal.refreshAll(kickerVolt, kickerCurrent);

    inputs.KickerAppliedVolts = kickerVolt.getValueAsDouble();
    inputs.KickerAppliedAmps = kickerCurrent.getValueAsDouble();
  }

  @Override
  public void setKickerVoltage(double volts) {
    KickerMotor.setControl(kickerVoltageRequest.withOutput(volts));
  }
}
