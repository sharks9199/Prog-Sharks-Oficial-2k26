package frc.robot.subsystems.shooter.Turret;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;

    // Setpoints para debug
    public double setpointPositionRads = 0.0;
    public double setpointVelocityRadsPerSec = 0.0;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  /** Mover para um ângulo específico (Motion Magic / Profiled PID) */
  default void runSetpoint(Angle position) {}

  /** Controle manual/teste */
  default void runVolts(Voltage volts) {}

  /** Ajuste de PID em tempo real */
  default void setPID(double p, double i, double d) {}

  default void stop() {}
}