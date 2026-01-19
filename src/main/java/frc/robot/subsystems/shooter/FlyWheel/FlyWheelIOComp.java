package frc.robot.subsystems.shooter.FlyWheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.util.Units; // Importante para conversões se necessário

public class FlyWheelIOComp implements FlyWheelIO {

  // --- HARDWARE ---
  private final TalonFX FWMotor;

  // --- REQUESTS ---
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public FlyWheelIOComp(int motorId) {
    FWMotor = new TalonFX(motorId);

    var config = new TalonFXConfiguration();

    // 1. Limite de Corrente (40A)
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // 2. Modo Coast (Roda solta)
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // 3. PID (Slot 0)
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12; 

    FWMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    // --- ATUALIZAÇÃO ---
    // Removemos 'positionRads' pois não definimos na Interface (não é útil para Shooter)
    
    // Velocidade: Rotations/Sec -> Radians/Sec
    // O Kraken retorna 'StatusSignal', usamos getValueAsDouble()
    inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(FWMotor.getVelocity().getValueAsDouble() * 60); 
    // ou mais direto:
    // inputs.velocityRadsPerSec = Units.rotationsToRadians(FWMotor.getVelocity().getValueAsDouble()); // Se for RPS

    // Para garantir a unidade certa (RPS -> Rad/s):
    inputs.velocityRadsPerSec = edu.wpi.first.math.util.Units.rotationsToRadians(FWMotor.getVelocity().getValueAsDouble());

    // Dados Elétricos
    inputs.appliedVolts = FWMotor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = FWMotor.getSupplyCurrent().getValueAsDouble();
    
    // Removemos 'torqueCurrentAmps' pois também não definimos na Interface
  }

  @Override
  public void runVolts(Voltage volts) {
    FWMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    // O Kraken espera Rotações por Segundo (RPS)
    double targetRPS = velocity.in(RotationsPerSecond);
    FWMotor.setControl(velocityRequest.withVelocity(targetRPS));
  }

  @Override
  public void setPID(double p, double i, double d) {
    var config = new Slot0Configs();
    config.kP = p;
    config.kI = i;
    config.kD = d;
    FWMotor.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
    FWMotor.setControl(voltageRequest.withOutput(0));
  }
}