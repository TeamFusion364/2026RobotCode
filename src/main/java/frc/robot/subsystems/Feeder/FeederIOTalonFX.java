package frc.robot.subsystems.Feeder;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Robot;

/** TalonFX-based Feeder IO implementation */
public class FeederIOTalonFX implements FeederIO {
  private final TalonFX FeederMotor = new TalonFX(Constants.feeder.indexerID);
  private final TalonFX FeederFollower = new TalonFX(Constants.feeder.indexerFollowerID);

  private final StatusSignal<Voltage> feederVolt = FeederMotor.getMotorVoltage();
  private final StatusSignal<Current> feederCurrent = FeederMotor.getSupplyCurrent();

  private VoltageOut feederVoltageRequest = new VoltageOut(0);

  public FeederIOTalonFX() {
    tryUntilOk(
        5, () -> FeederMotor.getConfigurator().apply(Robot.hardwareConfigs.feederConfig, 0.25));
    tryUntilOk(
        5, () -> FeederFollower.getConfigurator().apply(Robot.hardwareConfigs.feederConfig, 0.25));
    FeederFollower.setControl(new Follower(FeederMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, feederVolt, feederCurrent);
    FeederMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(feederVolt, feederCurrent);

    inputs.FeederAppliedVolts = feederVolt.getValueAsDouble();
    inputs.FeederAppliedAmps = feederCurrent.getValueAsDouble();
  }

  @Override
  public void setFeederVoltage(double volts) {
    FeederMotor.setControl(feederVoltageRequest.withOutput(-volts));
  }
}
