package frc.robot.subsystems.shooter.FlyWheel;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface FlyWheelIO {

  @AutoLog
  public static class FlyWheelIOInputs {
    // --- Motor Principal (Flywheel) ---
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;

    // --- O ERRO ESTAVA AQUI: Faltava esta variável ---
    public double setpointVelocityRadsPerSec = 0.0;

    // --- Motor Centrífuga ---
    public double centrifugeVelocityRpm = 0.0;
    public double centrifugeAppliedVolts = 0.0;
    public double centrifugeCurrentAmps = 0.0;

    // --- Motor Feeder ---
    public double feederVelocityRpm = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
  }

  public default void updateInputs(FlyWheelIOInputs inputs) {
  }

  public default void runVolts(Voltage volts) {
  }

  public default void runVelocity(AngularVelocity velocity) {
  }

  public default void setPID(double p, double i, double d) {
  }

  public default void runCentrifugeVolts(double volts) {
  }

  public default void runCentrifugeVelocity(AngularVelocity velocity) {
  }

  public default void runFeederVolts(double volts) {
  }

  public default void runFeederVelocity(AngularVelocity velocity) {
  }

  public default void stop() {
  }
}