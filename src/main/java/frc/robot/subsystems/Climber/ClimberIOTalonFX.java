package frc.robot.subsystems.Climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Robot;

/** TalonFX-based Climber IO implementation */
public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climberMotor = new TalonFX(Constants.Climber.climberID);

  private final StatusSignal<Voltage> climberVolt = climberMotor.getMotorVoltage();
  private final StatusSignal<Current> climberCurrent = climberMotor.getSupplyCurrent();
  private final StatusSignal<Angle> climberPosition = climberMotor.getPosition();

  private PositionVoltage climberPositionRequest = new PositionVoltage(0);
  private VoltageOut climberVoltageRequest = new VoltageOut(0);

  public ClimberIOTalonFX() {
    tryUntilOk(
        5, () -> climberMotor.getConfigurator().apply(Robot.hardwareConfigs.climberConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, climberVolt, climberCurrent, climberPosition);
    climberMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(climberVolt, climberCurrent, climberPosition);

    inputs.ClimberAppliedVolts = climberVolt.getValueAsDouble();
    inputs.ClimberAppliedAmps = climberCurrent.getValueAsDouble();
    inputs.ClimberPosition = climberPosition.getValueAsDouble();
  }

  @Override
  public void setClimberVoltage(double volts) {
    climberMotor.setControl(climberVoltageRequest.withOutput(volts));
  }

  @Override
  public void setClimberPosition(double rotations) {
    climberMotor.setControl(climberPositionRequest.withPosition(rotations));
  }
}
