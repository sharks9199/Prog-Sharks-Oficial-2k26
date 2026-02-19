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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;

public class TurretIOComp implements TurretIO {

  private final TalonFX talon;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final DigitalInput max2LimitSwitch = new DigitalInput(1);
  private final DigitalInput max1LimitSwitch = new DigitalInput(2);

  public TurretIOComp() {
    talon = new TalonFX(TurretConstants.kTurretMotorID); 
    
    var config = new TalonFXConfiguration();

    // --- 1. CONFIGURAÇÃO GERAL ---
    config.CurrentLimits.SupplyCurrentLimit = 30.0; 
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    config.Feedback.SensorToMechanismRatio = 10.0;

    // --- 2. PID (SLOT 0) ---
    config.Slot0.kP = 65.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.5;

    config.Slot0.kS = 0.15; 
    config.Slot0.kV = 0.12; 

    config.MotionMagic.MotionMagicCruiseVelocity = 1.5; 
    config.MotionMagic.MotionMagicAcceleration = 2.0;
    config.MotionMagic.MotionMagicJerk = 0;

    talon.getConfigurator().apply(config);
    talon.setPosition(0.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(talon.getPosition().getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(talon.getVelocity().getValueAsDouble());
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();

    inputs.initialLimitHit = max1LimitSwitch.get();
    inputs.max1LimitHit = max1LimitSwitch.get();
    inputs.max2LimitHit = max2LimitSwitch.get();
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
  public void resetEncoder() {
    talon.setPosition(0);
  }

  @Override
  public void setEncoderPosition(Angle position) {
    talon.setPosition(position.in(Rotations));
  }

  @Override
  public void stop() {
    talon.setControl(voltageRequest.withOutput(0));
  }
}