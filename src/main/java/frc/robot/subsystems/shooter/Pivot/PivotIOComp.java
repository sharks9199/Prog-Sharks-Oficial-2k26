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
import frc.robot.subsystems.shooter.ShooterConstants.PivotConstants;

public class PivotIOComp implements PivotIO {

  private final TalonFX talon;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final double kStartAngleDegrees = 0.0; 

  public PivotIOComp() {
    talon = new TalonFX(PivotConstants.kPivotMotor);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 45.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

    config.Feedback.SensorToMechanismRatio = 50.0; 

    // --- PID ---
    config.Slot0.kP = 95.0; 
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.Slot0.kG = 0.2; 
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; 

    // --- MOTION MAGIC ---
    config.MotionMagic.MotionMagicCruiseVelocity = 1.5;
    config.MotionMagic.MotionMagicAcceleration = 2.0;

    talon.getConfigurator().apply(config);

    talon.setPosition(Units.degreesToRotations(kStartAngleDegrees));
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
  public void resetEncoder() {
    talon.setPosition(0);
  }

  @Override
  public void stop() {
    talon.setControl(voltageRequest.withOutput(0));
  }
}