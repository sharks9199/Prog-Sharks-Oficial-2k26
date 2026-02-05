package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
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

    public static final double kMaxSpeedMps = 4.0;
    public static final double kMaxAccelMpsSq = 3.5;

    public static final PathConstraints constraints = new PathConstraints(
        kMaxSpeedMps,
        kMaxAccelMpsSq,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540));

    public static final PathConstraints constraintsAuto = new PathConstraints(
        kMaxSpeedMps,
        kMaxAccelMpsSq,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540));
  }

  public static final class OIConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kSecondDriverControllerPort = 1;

    public static final int kResetFrontIdx = 1;
    public static final int kThroughtTrenchIdx = 2;
    public static final int kAutoAimIdx = 3;
    public static final int kResetTurretEncoderIdx = 4;
    public static final int kIntakeIdx = 6;
    public static final int kTurretToLeftPOV = 270;
    public static final int kTurretToRightPOV = 90;
    public static final int kLowerIntakeButtonIdx = 180;
    public static final int kRaiseIntakeButtonIdx = 0;
  }
}