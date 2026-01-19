package frc.robot.subsystems.shooter.Pivot;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    // Para o Log: Use double (Radianos, Volts, Amperes)
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    
    // Setpoints (útil para ver no gráfico se o robô está seguindo a meta)
    public double setpointPositionRads = 0.0;
    public double setpointVelocityRadsPerSec = 0.0;
  }

  /** Atualiza os inputs */
  default void updateInputs(PivotIOInputs inputs) {}

  /** Manda o motor ir para um ângulo (Usando Units!) */
  default void runSetpoint(Angle position) {}

  /** Manda voltagem direta */
  default void runVolts(Voltage volts) {}

  /** Configura PID (chamado no inicio do codigo) */
  default void setPID(double p, double i, double d) {}

  default void stop() {}
}