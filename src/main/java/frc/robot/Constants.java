package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL, SIM, REPLAY
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMps = 4.0;
    public static final double kMaxAccelMpsSq = 4.0;

    public static final PathConstraints constraints = new PathConstraints(
        kMaxSpeedMps, kMaxAccelMpsSq,
        Units.degreesToRadians(360), Units.degreesToRadians(540));

    public static final PathConstraints constraintsAuto = constraints;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondDriverControllerPort = 1;

    // ---DRIVER BINDINGS---
    public static final int kResetFrontIdx = 3; // X
    public static final int kThroughtTrenchIdx = 1; // A
    public static final int kIntakeIdx = 2; // B
    public static final int kAutoClimbingIdx = 4; // Y

    public static final int kAmpIdx = 5; // LB (Cuspir)
    public static final int kAutoAimIdx = 6; // RB
    public static final int KThroughtOtherAllianceTrench = 7; // LT (Resetar Pivot separado)
    public static final int kShootidx = 8; // RT (Atirar)

    public static final int kTurretToLeftPOV = 270;
    public static final int kTurretToRightPOV = 90;

    // ---OPERATOR BINDINGS---

    public static final int kResetTurretEncoderIdx = 4; // Y
    public static final int kResetPivotIdx = 7; // LT (Resetar Pivot separado)   
    public static final int kLowerIntakeButtonIdx = 180;
    public static final int kRaiseIntakeButtonIdx = 0; 
  }

  public static final class LEDConstants {

    public static final int kLEOPort = 8;
    public static final int kTotalLength = 300;

    public static final int kAllianceStart = 0;
    public static final int kAllianceEnd = 99;

    public static final int kShooterStart = 100;
    public static final int kShooterEnd = 199;

    public static final int kBoostStart = 200;
    public static final int kBoostEnd = 299; // Tem que ser kTotalLength - 1
    
    // =========================== LED OFF =================================
    public static final LEDPattern ledOff = LEDPattern.solid(Color.kBlack);
    // =======================================================================

    // =========================== Blinking Green ===========================
    public static LEDPattern baseGreen = LEDPattern.solid(Color.kGreen);
    public static LEDPattern baseBlack = LEDPattern.solid(Color.kBlack);
    // =======================================================================

    // =========================== Blinking Red ===========================
    public static LEDPattern baseRed = LEDPattern.solid(Color.kRed);
    public static LEDPattern blinkingRed = baseRed.blink(Seconds.of(0.1));
    // =======================================================================

    // =========================== Blinking Blue ============================
    public static LEDPattern baseBlue = LEDPattern.solid(Color.kBlue);
    public static LEDPattern blinkingBlue = baseBlue.blink(Seconds.of(0.1));
    // =======================================================================

    // =========================== Blinking Yellow ==========================
    public static LEDPattern baseYellow = LEDPattern.solid(Color.kYellow);
    public static LEDPattern blinkingYellow = baseYellow.blink(Seconds.of(0.1));
    // =======================================================================

    // ================================ Turbo ===============================
    public static double speed = 0;
    public static final LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed,
        Color.kOrangeRed);
    public static LEDPattern mask = LEDPattern.progressMaskLayer(() -> speed);
    public static LEDPattern turboDisplay = base.mask(mask);
    // =======================================================================

    public static boolean turboEffect = true;
  }
}