package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

    public static final class AutoConstants {
        public static PathConstraints constraints = new PathConstraints(1.0, 1.0, 
        Units.degreesToRadians(180), Units.degreesToRadians(180));
        
        public static PathConstraints constraintsAuto = new PathConstraints(0.5, 1.0, 
        Units.degreesToRadians(180), Units.degreesToRadians(180));
}
}
