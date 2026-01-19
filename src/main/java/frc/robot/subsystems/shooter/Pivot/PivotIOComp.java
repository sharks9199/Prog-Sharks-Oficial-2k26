package frc.robot.subsystems.shooter.Pivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class PivotIOComp implements PivotIO {

  private final TalonFX talon;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public PivotIOComp(int motorId) {
    talon = new TalonFX(motorId);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 20.0; // Hood gasta pouca corrente
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
    
    // VERIFICAR DIREÇÃO: O Hood sobe ou desce com positivo?
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

    // REDUÇÃO: Verifique a redução exata do seu Hood
    config.Feedback.SensorToMechanismRatio = 50.0; 

    // --- PID ---
    config.Slot0.kP = 20.0; // P alto para corrigir pequenos erros de ângulo
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    
    // --- GRAVIDADE ---
    // Mesmo sendo semicirculo, Arm_Cosine funciona se ele girar no eixo.
    // Se for fuso (parafuso sem fim), use GravityType.Elevator_Static
    config.Slot0.kG = 0.2; 
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; 

    // --- MOTION MAGIC ---
    config.MotionMagic.MotionMagicCruiseVelocity = 2.0; // Rápido
    config.MotionMagic.MotionMagicAcceleration = 4.0;

    // --- LIMITES DE SEGURANÇA (IMPORTANTE PARA HOOD) ---
    // Hoods geralmente quebram se passarem do ponto.
    // Exemplo: 0 graus (fechado) a 45 graus (aberto maximo)
    // 45 graus = 0.125 rotações
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(45); 
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;  
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    talon.getConfigurator().apply(config);
    
    // Assume que inicia fechado (zero)
    talon.setPosition(0.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
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
    config.kP = p;
    config.kI = i;
    config.kD = d;
    talon.getConfigurator().apply(config); 
  }

  @Override
  public void stop() {
    talon.setControl(voltageRequest.withOutput(0));
  }
}