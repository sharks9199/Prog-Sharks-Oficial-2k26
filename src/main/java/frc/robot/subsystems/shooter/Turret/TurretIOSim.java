package frc.robot.subsystems.shooter.Turret;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO {

  // --- FÍSICA ---
  private final SingleJointedArmSim sim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1), 
      50.0,
      0.01,
      0.2,
      Math.toRadians(-90),
      Math.toRadians(90),
      false,
      Math.toRadians(0)
  );

  private final ProfiledPIDController controller = new ProfiledPIDController(
      5.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(
          Units.rotationsToRadians(1.5),
          Units.rotationsToRadians(3.0)
      )
  );

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
      0.0,  // Simulação perfeita não tem muito atrito estático
      0.1,  // kV
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
    double pidOutput = controller.calculate(sim.getAngleRads(), targetPosition.in(Radians));
    
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