package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Intake.IntakeConstants.intakeConstants;

public class IntakeIOComp implements IntakeIO {

    private final TalonFX rotationMotor;
    private final TalonFX wheelMotor;

    public IntakeIOComp() {
        rotationMotor = new TalonFX(intakeConstants.RotationMotorID);
        wheelMotor = new TalonFX(intakeConstants.WheelMotorID);

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = intakeConstants.RotationGearRatio;
        config.Slot0.kP = 45.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.5;

        config.Slot0.kS = 0.15;
        config.Slot0.kV = 0.12;

        rotationMotor.getConfigurator().apply(config);
        rotationMotor.setPosition(0.0);

        wheelMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rotationPosition = rotationMotor.getPosition().getValueAsDouble();
        inputs.rotationVelocity = rotationMotor.getVelocity().getValueAsDouble();
        inputs.rotationAppliedVolts = rotationMotor.getMotorVoltage().getValueAsDouble();
        inputs.rotationCurrentAmps = rotationMotor.getSupplyCurrent().getValueAsDouble();

        inputs.wheelVelocity = wheelMotor.getVelocity().getValueAsDouble();
        inputs.wheelAppliedVolts = wheelMotor.getMotorVoltage().getValueAsDouble();
        inputs.wheelCurrentAmps = wheelMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getPosition() {
        return rotationMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getSpeed() {
        return rotationMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getSetpoint() {
        return intakeConstants.intakeSetpoint;
    }

    @Override
    public void setStopMode() {
        rotationMotor.set(0);
    }

    @Override
    public void setPlanetary(double speed) {
        speed = MathUtil.clamp(speed, -intakeConstants.IntakeMaxSpeed, intakeConstants.IntakeMaxSpeed);
        rotationMotor.set(speed);
    }

    @Override
    public void setIntake(double speed) {
        wheelMotor.set(speed);
    }

    @Override
    public void changeSetpoint(double setpoint) {
        intakeConstants.intakeSetpoint = setpoint;
    }
}