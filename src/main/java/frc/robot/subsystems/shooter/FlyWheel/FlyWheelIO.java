package frc.robot.subsystems.shooter.FlyWheel;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.*;

public interface FlyWheelIO {
  @AutoLog
  public static class FlyWheelIOInputs {
    // Inputs puros (double) para o AdvantageScope amar
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    
    // Para comparar no gráfico: O que eu queria vs O que eu consegui
    public double setpointVelocityRadsPerSec = 0.0;
  }

  /** Atualiza leituras */
  default void updateInputs(FlyWheelIOInputs inputs) {}

  /** Rodar usando controle de malha fechada (PID + FF) */
  default void runVelocity(AngularVelocity velocity) {}

  /** Rodar usando voltagem pura (teste ou SysId) */
  default void runVolts(Voltage volts) {}

  /** Configura as constantes de PID */
  default void setPID(double p, double i, double d) {}

  /** Para o motor */
  default void stop() {}
}