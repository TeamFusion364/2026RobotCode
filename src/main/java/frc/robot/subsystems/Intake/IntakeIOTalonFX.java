package frc.robot.subsystems.Intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Robot;

/** TalonFX-based Intake IO implementation */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor = new TalonFX(Constants.intake.intakeID);
  private final TalonFX strokerMotor = new TalonFX(Constants.intake.strokerID);

  private final StatusSignal<AngularVelocity> intakeVel = intakeMotor.getVelocity();
  private final StatusSignal<Voltage> intakeVolt = intakeMotor.getMotorVoltage();
  private final StatusSignal<Current> intakeCurrent = intakeMotor.getSupplyCurrent();

  private final StatusSignal<Voltage> strokerVolt = strokerMotor.getMotorVoltage();
  private final StatusSignal<Current> strokerCurrent = strokerMotor.getSupplyCurrent();
  private final StatusSignal<Angle> strokerPosition = strokerMotor.getPosition();

  private VoltageOut intakeVoltageRequest = new VoltageOut(0);
  private MotionMagicVelocityVoltage intakeVelocityRequest = new MotionMagicVelocityVoltage(0);
  private PositionVoltage strokerPositionRequest = new PositionVoltage(0);
  private VoltageOut strokerVoltageRequest = new VoltageOut(0);

  public IntakeIOTalonFX() {
    tryUntilOk(
        5, () -> intakeMotor.getConfigurator().apply(Robot.hardwareConfigs.intakeConfig, 0.25));
    tryUntilOk(
        5, () -> strokerMotor.getConfigurator().apply(Robot.hardwareConfigs.strokerConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakeVel, intakeVolt, intakeCurrent, strokerVolt, strokerCurrent, strokerPosition);
    intakeMotor.optimizeBusUtilization();
    strokerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakeVel, intakeVolt, intakeCurrent, strokerVolt, strokerCurrent, strokerPosition);

    inputs.IntakeVelocityRPS = intakeVel.getValueAsDouble();
    inputs.IntakeAppliedVolts = intakeVolt.getValueAsDouble();
    inputs.IntakeAppliedAmps = intakeCurrent.getValueAsDouble();

    inputs.StrokerAppliedVolts = strokerVolt.getValueAsDouble();
    inputs.StrokerAppliedAmps = strokerCurrent.getValueAsDouble();
    // Convert stroker position to inches for the public API. The underlying sensor value is
    // assumed to be in meters (configure sensor/feedback ratio accordingly in HardwareConfigs),
    // so convert meters -> inches for external use.
    inputs.StrokerPositionInches =
        edu.wpi.first.math.util.Units.metersToInches(strokerPosition.getValueAsDouble());
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setControl(intakeVoltageRequest.withOutput(volts));
  }

  @Override
  public void setIntakeRPS(double rps) {
    intakeMotor.setControl(intakeVelocityRequest.withVelocity(rps));
  }

  @Override
  public void setStrokerVoltage(double volts) {
    strokerMotor.setControl(strokerVoltageRequest.withOutput(volts));
  }

  @Override
  public void setStrokerPosition(double inches) {
    strokerMotor.setControl(strokerPositionRequest.withPosition(inches));
  }
}
