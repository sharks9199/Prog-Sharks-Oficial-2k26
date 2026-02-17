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

    public double setpointPositionRads = 0.0;
    public double setpointVelocityRadsPerSec = 0.0;

    public boolean initialLimitHit = false;
    public boolean max1LimitHit = false;
    public boolean max2LimitHit = false;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void runSetpoint(Angle position) {}

  default void runVolts(Voltage volts) {}

  default void setPID(double p, double i, double d) {}

  default void resetEncoder() {}

  default void setEncoderPosition(Angle position) {}

  default void stop() {}
}