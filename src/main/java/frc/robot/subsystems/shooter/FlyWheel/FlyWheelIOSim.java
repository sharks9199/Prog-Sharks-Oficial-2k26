package frc.robot.subsystems.shooter.FlyWheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId; // <--- NOVO IMPORT NECESSÁRIO
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlyWheelIOSim implements FlyWheelIO {

  // --- FÍSICA ---
  // A melhor forma de instanciar é criando o Sistema Linear (Plant) explicitamente.
  // Isso resolve qualquer erro de construtor "undefined".
  private final FlywheelSim sim = new FlywheelSim(
      // 1. Cria o modelo matemático (Motor, Inércia J, Redução)
      // Nota: A ordem aqui é (Motor, Inércia, Redução)
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.004, 1.0),
      
      // 2. Passa o motor novamente (para cálculos de corrente)
      DCMotor.getKrakenX60(1),
      
      // 3. Passa a redução novamente
      1.0
  );

  // --- CONTROLE (PID + FF) ---
  private final PIDController pid = new PIDController(0.1, 0.0, 0.0);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.05, 0.12, 0.01);

  private double appliedVolts = 0.0;
  private double currentSetpointRadsPerSec = 0.0; 

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    sim.update(0.02);

    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.setpointVelocityRadsPerSec = currentSetpointRadsPerSec;
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

  @Override
  public void setPID(double p, double i, double d) {
    pid.setPID(p, i, d);
  }

  @Override
  public void stop() {
    runVolts(Volts.of(0));
    currentSetpointRadsPerSec = 0.0;
  }
}