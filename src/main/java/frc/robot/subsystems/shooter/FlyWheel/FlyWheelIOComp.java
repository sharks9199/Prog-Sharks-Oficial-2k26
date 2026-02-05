package frc.robot.subsystems.shooter.FlyWheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.shooter.ShooterConstants.FlyWheelConstants;
import edu.wpi.first.math.util.Units;

public class FlyWheelIOComp implements FlyWheelIO {

  private final TalonFX FWMotor;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public FlyWheelIOComp() {
    FWMotor = new TalonFX(FlyWheelConstants.kFWMotor);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 0.1; 
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12; 

    FWMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    inputs.velocityRadsPerSec = Units.rotationsToRadians(FWMotor.getVelocity().getValueAsDouble());

    inputs.appliedVolts = FWMotor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = FWMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runVolts(Voltage volts) {
    FWMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    double targetRPS = velocity.in(RotationsPerSecond);

    FWMotor.setControl(velocityRequest.withVelocity(targetRPS));
  }

  @Override
  public void setPID(double p, double i, double d) {
    var config = new Slot0Configs();

    FWMotor.getConfigurator().refresh(config);
    
    // Atualiza só o PID
    config.kP = p;
    config.kI = i;
    config.kD = d;
    
    // Aplica de volta
    FWMotor.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
    FWMotor.setControl(voltageRequest.withOutput(0));
  }
}