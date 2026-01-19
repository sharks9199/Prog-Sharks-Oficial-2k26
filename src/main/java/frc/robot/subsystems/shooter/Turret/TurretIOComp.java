package frc.robot.subsystems.shooter.Turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class TurretIOComp implements TurretIO {

  private final TalonFX talon;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public TurretIOComp(int motorId) {
    talon = new TalonFX(motorId);
    var config = new TalonFXConfiguration();

    // 1. HARDWARE
    config.CurrentLimits.SupplyCurrentLimit = 30.0; // Turret não precisa de tanta força
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    // BRAKE É ESSENCIAL: Segura o turret no lugar quando parado
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // Verifique: Positivo gira pra esquerda ou direita? (CCW+)
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

    // REDUÇÃO: Exemplo 100:1 (ajuste para sua realidade)
    config.Feedback.SensorToMechanismRatio = 100.0;

    // 2. PID (Sem kG pois não tem gravidade)
    config.Slot0.kP = 12.0; 
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    
    // kS: Ajuda a vencer o atrito do rolamento (Static Friction)
    config.Slot0.kS = 0.15; 
    config.Slot0.kV = 0.12; 

    // 3. MOTION MAGIC (Suavidade)
    config.MotionMagic.MotionMagicCruiseVelocity = 1.5; // 1.5 voltas/s
    config.MotionMagic.MotionMagicAcceleration = 3.0;

    // 4. SOFT LIMITS (NÃO ARRANQUE SEUS CABOS!)
    // Exemplo: Turret gira 90 graus pra cada lado (Total 180)
    // 90 graus = 0.25 rotações
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25; 
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.25;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    talon.getConfigurator().apply(config);
    
    // IMPORTANTE: Turret precisa começar alinhado (frente) ou usar Sensor Absoluto
    talon.setPosition(0.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(talon.getPosition().getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(talon.getVelocity().getValueAsDouble());
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runSetpoint(Angle position) {
    double targetRotations = position.in(Rotations);
    talon.setControl(positionRequest.withPosition(targetRotations));
  }

  @Override
  public void runVolts(Voltage volts) {
    talon.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }

  @Override
  public void setPID(double p, double i, double d) {
    var config = new Slot0Configs();
    config.kP = p; config.kI = i; config.kD = d;
    talon.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
    talon.setControl(voltageRequest.withOutput(0));
  }
}