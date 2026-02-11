package frc.robot.subsystems.shooter.Pivot;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;

    public double setpointPositionRads = 0.0;
    public double setpointVelocityRadsPerSec = 0.0;
  }

  default void updateInputs(PivotIOInputs inputs) {
  }

  default void runSetpoint(Angle position) {
  }

  default void runVolts(Voltage volts) {
  }

  default void resetEncoder() {
  }

  default void setPID(double p, double i, double d) {
  }

  default void stop() {
  }
}