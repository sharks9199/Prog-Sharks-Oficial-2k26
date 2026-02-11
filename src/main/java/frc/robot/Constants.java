package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL, SIM, REPLAY
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMps = 4.0;
    public static final double kMaxAccelMpsSq = 3.5;

    public static final PathConstraints constraints = new PathConstraints(
        kMaxSpeedMps, kMaxAccelMpsSq,
        Units.degreesToRadians(360), Units.degreesToRadians(540));

    public static final PathConstraints constraintsAuto = constraints; 
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondDriverControllerPort = 1;

    // ---DRIVER BINDINGS---
    public static final int kResetFrontIdx = 1;                       // X
    public static final int kThroughtTrenchIdx = 2;                   // A
    public static final int kIntakeIdx = 3;                           // B
    public static final int kAutoClimbingIdx = 4;                     // Y

    public static final int kAmpIdx = 5;                              // LB (Cuspir)
    public static final int kAutoAimIdx = 6;                          // RB
    public static final int KThroughtOtherAllianceTrench = 7;         // LT (Resetar Pivot separado)
    public static final int kShootidx = 8;                            // RT (Atirar)

    public static final int kTurretToLeftPOV = 270;
    public static final int kTurretToRightPOV = 90;
    public static final int kLowerIntakeButtonIdx = 180;
    public static final int kRaiseIntakeButtonIdx = 0;

    // ---OPERATOR BINDINGS---

    public static final int kResetTurretEncoderIdx = 4;                // Y
    public static final int kResetPivotIdx = 7;                        // LT (Resetar Pivot separado)
  }
}