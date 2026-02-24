package frc.robot.subsystems.shooter.FlyWheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.shooter.ShooterConstants.FlyWheelConstants;
import edu.wpi.first.math.util.Units;

public class FlyWheelIOComp implements FlyWheelIO {

  // Motores
  private final TalonFX FWMotor;
  private final TalonFX centrifugeMotor;
  private final TalonFX feederMotor;

  // Requests de Controle
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  
  private final VoltageOut centrifugeRequest = new VoltageOut(0);
  private final VelocityVoltage centrifugeVelocityRequest = new VelocityVoltage(0);

  private final VoltageOut feederRequest = new VoltageOut(0);
  private final VelocityVoltage feederVelocityRequest = new VelocityVoltage(0);

  public FlyWheelIOComp() {
    // -----------------------------------------------------------
    // 1. CONFIGURAÇÃO DO FLYWHEEL PRINCIPAL
    // -----------------------------------------------------------
    FWMotor = new TalonFX(FlyWheelConstants.kShooterFWMotor);
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 1.5; 
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12; 
    FWMotor.getConfigurator().apply(config);

    // -----------------------------------------------------------
    // 2. CONFIGURAÇÃO DA CENTRÍFUGA
    // -----------------------------------------------------------
    centrifugeMotor = new TalonFX(FlyWheelConstants.kCentrifugeMotor); 
    var centConfig = new TalonFXConfiguration();
    centConfig.CurrentLimits.SupplyCurrentLimit = 40.0; 
    centConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    centConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    centConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
    centConfig.Slot0.kP = 0.05;
    centConfig.Slot0.kI = 0.0;
    centConfig.Slot0.kD = 0.0;
    centConfig.Slot0.kV = 0.12;
    centrifugeMotor.getConfigurator().apply(centConfig);

    // -----------------------------------------------------------
    // 3. CONFIGURAÇÃO DO FEEDER
    // -----------------------------------------------------------
    feederMotor = new TalonFX(FlyWheelConstants.kFeederFWMotor);
    var feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.SupplyCurrentLimit = 50.0; 
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    feederConfig.Slot0.kP = 0.05;
    feederConfig.Slot0.kI = 0.0;
    feederConfig.Slot0.kD = 0.0;
    feederConfig.Slot0.kV = 0.12;
    feederMotor.getConfigurator().apply(feederConfig);
  }

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    // Flywheel
    inputs.velocityRadsPerSec = Units.rotationsToRadians(FWMotor.getVelocity().getValueAsDouble());
    inputs.appliedVolts = FWMotor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = FWMotor.getSupplyCurrent().getValueAsDouble();

    // Centrifuga
    inputs.centrifugeAppliedVolts = centrifugeMotor.getMotorVoltage().getValueAsDouble();
    inputs.centrifugeCurrentAmps = centrifugeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.centrifugeVelocityRpm = centrifugeMotor.getVelocity().getValueAsDouble();

    // Feeder
    inputs.feederAppliedVolts = feederMotor.getMotorVoltage().getValueAsDouble();
    inputs.feederCurrentAmps = feederMotor.getSupplyCurrent().getValueAsDouble();
    inputs.feederVelocityRpm = feederMotor.getVelocity().getValueAsDouble();
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
  public void runCentrifugeVolts(double volts) {
      centrifugeMotor.setControl(centrifugeRequest.withOutput(volts));
  }

  @Override
  public void runCentrifugeVelocity(AngularVelocity velocity) {
      double targetRPS = velocity.in(RotationsPerSecond);
      centrifugeMotor.setControl(centrifugeVelocityRequest.withVelocity(targetRPS));
  }

  @Override
  public void runFeederVolts(double volts) {
      feederMotor.setControl(feederRequest.withOutput(volts));
  }

  @Override
  public void runFeederVelocity(AngularVelocity velocity) {
      double targetRPS = velocity.in(RotationsPerSecond);
      feederMotor.setControl(feederVelocityRequest.withVelocity(targetRPS));
  }

  @Override
  public void setPID(double p, double i, double d) {
    var config = new Slot0Configs();
    FWMotor.getConfigurator().refresh(config);
    config.kP = p; config.kI = i; config.kD = d;
    FWMotor.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
    FWMotor.setControl(voltageRequest.withOutput(0));
    centrifugeMotor.setControl(centrifugeRequest.withOutput(0));
    feederMotor.setControl(feederRequest.withOutput(0));
  }
}