package frc.robot.subsystems.shooter.Turret;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO {

  // --- FÍSICA ---
  private final SingleJointedArmSim sim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1), 
      100.0,                   // Redução ALTA (Turrets precisam de força e precisão)
      0.5,                     // Inércia (Massa giratória do conjunto todo)
      0.3,                     // Comprimento (raio do turret)
      Math.toRadians(-180),    // Limite Mínimo (Ex: -180 graus)
      Math.toRadians(180),     // Limite Máximo (Ex: +180 graus)
      false,                   // <--- GRAVIDADE DESLIGADA (Horizontal)
      Math.toRadians(0)        // Começa no centro
  );

  // --- CONTROLE ---
  private final ProfiledPIDController controller = new ProfiledPIDController(
      15.0, 0.0, 0.0, // P
      new TrapezoidProfile.Constraints(
          Math.PI * 2,  // Max Vel (1 volta/s)
          Math.PI * 4   // Max Acel
      )
  );

  // Para Turret, usamos SimpleMotorFeedforward (kS, kV, kA) pois não tem kG (gravidade)
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
      0.1,  // kS (Atrito estático - precisa de força pra começar a girar)
      0.15, // kV
      0.01  // kA
  );

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sim.update(0.02);

    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    
    inputs.setpointPositionRads = controller.getSetpoint().position;
    inputs.setpointVelocityRadsPerSec = controller.getSetpoint().velocity;
  }

  @Override
  public void runSetpoint(Angle targetPosition) {
    // PID calcula a correção
    double pidOutput = controller.calculate(sim.getAngleRads(), targetPosition.in(Radians));
    
    // Feedforward calcula a física (sem gravidade)
    double ffOutput = ff.calculate(controller.getSetpoint().velocity);

    runVolts(Volts.of(pidOutput + ffOutput));
  }

  @Override
  public void runVolts(Voltage volts) {
    appliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPID(double p, double i, double d) {
    controller.setPID(p, i, d);
  }

  @Override
  public void stop() {
    runVolts(Volts.of(0));
  }
}