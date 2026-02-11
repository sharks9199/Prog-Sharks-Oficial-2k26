package frc.robot.subsystems.shooter.FlyWheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlyWheelIOSim implements FlyWheelIO {

  // --- Simulação do Motor Principal ---
  private final FlywheelSim sim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.004, 1.0),
      DCMotor.getKrakenX60(1),
      1.0
  );
  private final PIDController pid = new PIDController(0.1, 0.0, 0.0);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.05, 0.12, 0.01);

  private double appliedVolts = 0.0;
  private double currentSetpointRadsPerSec = 0.0; 

  // --- Simulação Simplificada para Feeder e Centrífuga ---
  // (Apenas para ver se está rodando no Dashboard, sem física complexa)
  private double centrifugeVolts = 0.0;
  private double feederVolts = 0.0;

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    // Atualiza Física do Flywheel
    sim.update(0.02);

    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.setpointVelocityRadsPerSec = currentSetpointRadsPerSec;

    // Atualiza Feeders (Simulação básica: Voltagem * k = Velocidade)
    inputs.centrifugeAppliedVolts = centrifugeVolts;
    inputs.centrifugeVelocityRpm = centrifugeVolts * 500; // Valor fictício para visualização
    
    inputs.feederAppliedVolts = feederVolts;
    inputs.feederVelocityRpm = feederVolts * 500; // Valor fictício para visualização
  }

  @Override
  public void runVelocity(AngularVelocity targetVelocity) {
    currentSetpointRadsPerSec = targetVelocity.in(RadiansPerSecond);

    double ffVolts = ff.calculate(currentSetpointRadsPerSec);
    double pidVolts = pid.calculate(sim.getAngularVelocityRadPerSec(), currentSetpointRadsPerSec);

    runVolts(Volts.of(ffVolts + pidVolts));
  }

  @Override
  public void runVolts(Voltage volts) {
    appliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }

  // --- Implementação do Feeder e Centrífuga no Sim ---
  
  @Override
  public void runCentrifugeVolts(double volts) {
      this.centrifugeVolts = MathUtil.clamp(volts, -12, 12);
  }

  @Override
  public void runCentrifugeVelocity(AngularVelocity velocity) {
      // Simplesmente converte velocidade pedida em voltagem proporcional para simular
      runCentrifugeVolts(velocity.in(RotationsPerSecond) * 0.1); 
  }

  @Override
  public void runFeederVolts(double volts) {
      this.feederVolts = MathUtil.clamp(volts, -12, 12);
  }

  @Override
  public void runFeederVelocity(AngularVelocity velocity) {
      runFeederVolts(velocity.in(RotationsPerSecond) * 0.1);
  }

  @Override
  public void setPID(double p, double i, double d) {
    pid.setPID(p, i, d);
  }

  @Override
  public void stop() {
    runVolts(Volts.of(0));
    currentSetpointRadsPerSec = 0.0;
    centrifugeVolts = 0;
    feederVolts = 0;
  }
}