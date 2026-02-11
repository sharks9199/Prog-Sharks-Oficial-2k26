package frc.robot.subsystems.shooter.Pivot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    
  private final SingleJointedArmSim sim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1),  
      50.0,
      0.01,
      0.15,
      Math.toRadians(0),
      Math.toRadians(60),
      true,
      Math.toRadians(0)
  );

  // --- CONTROLE ---
  private final ProfiledPIDController controller = new ProfiledPIDController(
      20.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(
          Math.PI * 4,
          Math.PI * 8
      )
  );
  
  private final ArmFeedforward ff = new ArmFeedforward(
      0,0,
      0.2,
      0.12,
      0.01 
  );

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
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
    double ffOutput = ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity);

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