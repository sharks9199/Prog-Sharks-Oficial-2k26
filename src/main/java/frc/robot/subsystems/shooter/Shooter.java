package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

import frc.robot.subsystems.shooter.Turret.TurretIO;
import frc.robot.subsystems.shooter.Turret.TurretIOInputsAutoLogged;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIO;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.Pivot.PivotIO;
import frc.robot.subsystems.shooter.Pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import edu.wpi.first.wpilibj.Timer;

public class Shooter extends SubsystemBase {

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final FlyWheelIO flywheelIO;
    private final FlyWheelIOInputsAutoLogged flywheelInputs = new FlyWheelIOInputsAutoLogged();
    //private final IntakeIO intakeIO;
    //private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Translation2d> targetSupplier;

    private double currentTurretTarget = 0.0;
    private double currentPivotTarget = 68.89;
    private double kPivotOffset = 27.0;

    private double currentFlywheelTargetRpm = 0.0;
    private double currentFeederTargetRpm = 0.0;
    private double currentCentrifugeTargetRpm = 0.0;
    private Translation2d lastTargetLocation = null;

    public static double kMinPivotAngle = 55.0;
    private double kMaxPivotAngle = 68.89;

    private double kShootRpm = 6000.0;
    private double kFeederShootRpm = 5000.0;
    private double kCentrifugeShootRpm = 4500.0;

    private double kSpitRpm = 1000.0;
    private double kFeederSpitRpm = 2000.0;
    private double kCentrifugeSpitRpm = 60.0;

    private double kTargetHeightRelative = 2.1;
    private double kTurretToleranceDeg = 1.0;
    private double kPivotToleranceDeg = 1.0;

    private double kFlywheelToleranceRpm = 50.0;

    private double kMinTurretAngle = -110.0;
    private double kMaxTurretAngle = 17.0;

    private double kRpmSlope = 255.0;
    private double kRpmIntercept = 2060.00;
    private double kMaxSafeRpm = 6000.0;

    private final double kGravity = 9.81;
    private final double kWheelRadiusMeters = Units.inchesToMeters(2.0);

    private double calculatedAutoAimRpm = 0.0;

    private boolean hasReachedSpeed = false;
    private boolean autoAimEnabled = false;
    private boolean flywheelWithinTolerance = false;

    private boolean testModeEnabled = false;
    private double testManualRpm = 4000.0;

    private final Timer intakeAgitatorTimer = new Timer();
    private boolean isIntakeAgitatorDown = false;

    public Shooter(TurretIO turretIO, PivotIO pivotIO, FlyWheelIO flywheelIO){//, IntakeIO intakeIO) {
        this.turretIO = turretIO;
        this.pivotIO = pivotIO;
        this.flywheelIO = flywheelIO;
        //this.intakeIO = intakeIO;

        SmartDashboard.putNumber("Tuning/Shooter/MinPivotAngle", kMinPivotAngle);
    }

    public void setupAutoAimReferences(Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.targetSupplier = targetSupplier;
    }

    private double convertRpmToMps(double rpm) {
        double angularVelocityRadPerSec = rpm * (2.0 * Math.PI / 60.0);
        double wheelSurfaceSpeedMps = angularVelocityRadPerSec * kWheelRadiusMeters;
        double noteExitSpeedMps = wheelSurfaceSpeedMps;
        return noteExitSpeedMps;
    }

    @Override
    public void periodic() {
        // kMinPivotAngle = SmartDashboard.getNumber("Tuning/Shooter/MinPivotAngle",
        // kMinPivotAngle);
        turretIO.updateInputs(turretInputs);
        pivotIO.updateInputs(pivotInputs);
        flywheelIO.updateInputs(flywheelInputs);
        //intakeIO.updateInputs(intakeInputs);

        Logger.processInputs("Shooter/Turret", turretInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);
        //Logger.processInputs("Shooter/Intake", intakeInputs);
        SmartDashboard.putNumber("Turret/CurrentAngle", getTurretPosition());
        SmartDashboard.putNumber("Pivot/CurrentAngle", getPivotPosition());

        if (autoAimEnabled && robotPoseSupplier != null && targetSupplier != null) {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d targetLocation = targetSupplier.get();

            double dist = getDistanceToTarget(robotPose, targetLocation);
            SmartDashboard.putNumber("Calibration/TEST_DistanceToTarget", dist);

            double rawTargetRpm = testModeEnabled ? testManualRpm : (kRpmSlope * dist) + kRpmIntercept;

            rawTargetRpm = MathUtil.clamp(rawTargetRpm, 0, kMaxSafeRpm);
            double targetRpm = rawTargetRpm;
            // System.out.println("Target RPM: " + targetRpm);

            double dynamicMps = convertRpmToMps(targetRpm);
            double targetPivot = calculatePivotAngleNumeric(dist, dynamicMps);
            double targetTurret = calculateTurretAngle(robotPose, targetLocation);

            setTurretSetpoint(targetTurret);
            setPivotPosition(targetPivot);
            // System.out.println("Target Pivot Angle: " + targetPivot);

            calculatedAutoAimRpm = targetRpm;
        }

        currentTurretTarget = MathUtil.clamp(currentTurretTarget, kMinTurretAngle, kMaxTurretAngle);
        turretIO.runSetpoint(Degrees.of(currentTurretTarget));

        currentPivotTarget = MathUtil.clamp(currentPivotTarget, kMinPivotAngle, kMaxPivotAngle);
        pivotIO.runSetpoint(Degrees.of(currentPivotTarget));
        System.out.println("Pivot Target: " + currentPivotTarget);
        // ----------------------------------------

        if (currentFlywheelTargetRpm < 10 && currentFeederTargetRpm < 10 && currentCentrifugeTargetRpm < 10) {
            flywheelIO.stop();
        } else {
            if (currentFlywheelTargetRpm >= 10)
                flywheelIO.runVelocity(RPM.of(currentFlywheelTargetRpm));

            if (currentFeederTargetRpm < 10)
                flywheelIO.runFeederVolts(0.0);
            else
                flywheelIO.runFeederVelocity(RPM.of(currentFeederTargetRpm));

            if (currentCentrifugeTargetRpm < 10)
                flywheelIO.runCentrifugeVolts(0.0);
            else
                flywheelIO.runCentrifugeVelocity(RPM.of(currentCentrifugeTargetRpm));
        }
    }

    public void setFlywheelVelocity(double rpm) {
        this.currentFlywheelTargetRpm = rpm;
    }

    public void runFeeder(double rpm) {
        this.currentFeederTargetRpm = rpm;
    }

    public void runCentrifuge(double rpm) {
        this.currentCentrifugeTargetRpm = rpm;
    }

    public double getFlywheelRpm() {
        return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
    }

    public boolean isFlywheelAtSpeed() {
        if (currentFlywheelTargetRpm < 500) {
            flywheelWithinTolerance = false;
            return false;
        }
        double error = Math.abs(getFlywheelRpm() - currentFlywheelTargetRpm);
        if (!flywheelWithinTolerance) {
            if (error <= kFlywheelToleranceRpm)
                flywheelWithinTolerance = true;
        } else {
            if (error > kFlywheelToleranceRpm * 4.0)
                flywheelWithinTolerance = false;
        }
        return flywheelWithinTolerance;
    }

    private Command runShootSequence(double manualFlywheelTargetRpm, double feederTargetRpm,
            double centrifugeTargetRpm){//, Intake intake) {
        return this.run(() -> {
            double activeRpm = autoAimEnabled ? calculatedAutoAimRpm : manualFlywheelTargetRpm;
            setFlywheelVelocity(activeRpm);

            if (!hasReachedSpeed && isFlywheelAtSpeed())
                hasReachedSpeed = true;

            if (hasReachedSpeed) {
                runFeeder(feederTargetRpm);
                runCentrifuge(centrifugeTargetRpm);
            } else {
                runFeeder(0.0);
                runCentrifuge(0.0);
            }

           /*  if (intake != null && intakeAgitatorTimer.hasElapsed(3.0)) {
                if (isIntakeAgitatorDown) {
                    intake.changeSetpoint(
                            frc.robot.subsystems.intake.IntakeConstants.intakeConstants.IntakeShootingPosition);
                } else {
                    intake.changeSetpoint(frc.robot.subsystems.intake.IntakeConstants.intakeConstants.CollectPosition);
                }
                isIntakeAgitatorDown = !isIntakeAgitatorDown;
                intakeAgitatorTimer.restart();
            } */

        }).beforeStarting(() -> {
            hasReachedSpeed = false;
            flywheelWithinTolerance = false;

            //intakeAgitatorTimer.restart();
            //isIntakeAgitatorDown = false;
        }).finallyDo(() -> {
            hasReachedSpeed = false;
            flywheelWithinTolerance = false;
            stop();

           /* intakeAgitatorTimer.stop();
            if (intake != null) {
                intake.changeSetpoint(frc.robot.subsystems.intake.IntakeConstants.intakeConstants.CollectPosition);
            }*/
        });
    }

    public Command shootCommand(Intake intake) {
        return runShootSequence(calculatedAutoAimRpm, kFeederShootRpm, kCentrifugeShootRpm);//, intake);
    }

    public Command spitCommand(Intake intake) {
        return runShootSequence(kSpitRpm, kFeederSpitRpm, kCentrifugeSpitRpm);//, intake);
    }

    public double getDistanceToTarget(Pose2d robotPose, Translation2d targetLocation) {
        return robotPose.getTranslation().getDistance(targetLocation);
    }

    public double calculateTurretAngle(Pose2d robotPose, Translation2d targetLocation) {
        this.lastTargetLocation = targetLocation;
        double angleRad = Math.atan2(targetLocation.getY() - robotPose.getY(),
                targetLocation.getX() - robotPose.getX());
        return MathUtil.inputModulus(robotPose.getRotation().getDegrees() - Math.toDegrees(angleRad), -180, 180);
    }

    public double calculatePivotAngleNumeric(double distanceMeters, double velocityMPS) {
        double v2 = Math.pow(velocityMPS, 2);
        double discriminant = Math.pow(v2, 2)
                - kGravity * (kGravity * Math.pow(distanceMeters, 2) + 2 * kTargetHeightRelative * v2);

        if (discriminant < 0)
            return kMaxPivotAngle;

        double safeDistanceForMath = Math.max(distanceMeters, 0.1);
        double idealAngle = Math
                .toDegrees(Math.atan((v2 - Math.sqrt(discriminant)) / (kGravity * safeDistanceForMath)));

        return idealAngle;
    }

    public void setTurretSetpoint(double degrees) {
        this.currentTurretTarget = degrees;
    }

    public double getTurretPosition() {
        return Units.radiansToDegrees(turretInputs.positionRads);
    }

    public void setPivotPosition(double degreesReal) {
        System.out.println(degreesReal);
        this.currentPivotTarget = degreesReal + kPivotOffset;
    }

    public double getPivotPosition() {
        return Units.radiansToDegrees(pivotInputs.positionRads);
    }

    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
    }

    public void stop() {
        currentFlywheelTargetRpm = 0.0;
        currentFeederTargetRpm = 0.0;
        currentCentrifugeTargetRpm = 0.0;
        flywheelIO.stop();
    }

    public void resetTurretEncoder() {
        turretIO.resetEncoder();
    }

    public void resetPivotEncoder() {
        pivotIO.resetEncoder();
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }
}