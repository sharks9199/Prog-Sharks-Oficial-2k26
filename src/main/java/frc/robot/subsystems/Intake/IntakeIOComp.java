package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder; // IMPORTANTE
import frc.robot.subsystems.intake.IntakeConstants.intakeConstants;

public class IntakeIOComp implements IntakeIO {

    private final TalonFX rotationMotor;
    private final TalonFX wheelMotor;
    private final DutyCycleEncoder absoluteEncoder;

    public IntakeIOComp() {
        rotationMotor = new TalonFX(intakeConstants.RotationMotorID);
        wheelMotor = new TalonFX(intakeConstants.WheelMotorID);
        
        absoluteEncoder = new DutyCycleEncoder(intakeConstants.ThroughBoreEncoderPort);

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        rotationMotor.getConfigurator().apply(config);
        wheelMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double rawPosition = 1.0 - absoluteEncoder.get();
        
        inputs.rotationPosition = rawPosition - intakeConstants.EncoderOffset;
        
        inputs.rotationVelocity = rotationMotor.getVelocity().getValueAsDouble() / 25.0;
        
        inputs.rotationAppliedVolts = rotationMotor.getMotorVoltage().getValueAsDouble();
        inputs.rotationCurrentAmps = rotationMotor.getSupplyCurrent().getValueAsDouble();

        inputs.wheelVelocity = wheelMotor.getVelocity().getValueAsDouble();
        inputs.wheelAppliedVolts = wheelMotor.getMotorVoltage().getValueAsDouble();
        inputs.wheelCurrentAmps = wheelMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getPosition() {
        // INVERTIDO AQUI TAMBÉM
        return (1.0 - absoluteEncoder.get()) - intakeConstants.EncoderOffset;
    }

    @Override
    public double getSpeed() {
        return rotationMotor.getVelocity().getValueAsDouble() / 25.0; 
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