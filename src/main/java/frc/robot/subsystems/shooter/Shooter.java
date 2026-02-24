package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class Shooter extends SubsystemBase {

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final FlyWheelIO flywheelIO;
    private final FlyWheelIOInputsAutoLogged flywheelInputs = new FlyWheelIOInputsAutoLogged();

    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Translation2d> targetSupplier;

    private double currentTurretTarget = 0.0;
    private double currentFlywheelTargetRpm = 0.0;
    private double currentFeederTargetRpm = 0.0;
    private double currentCentrifugeRpm = 60.0;
    private Translation2d lastTargetLocation = null;

    private double kMinPivotAngle = 50.0;
    private double kMaxPivotAngle = 68.89;
    private double kPivotEncoderOffset = 35.0;

    private double kShootRpm = 4000.0;
    private double kFeederShootRpm = 4000.0;
    private double kSpitRpm = 1000.0;
    private double kFeederSpitRpm = 2000.0;

    private double kTargetHeightRelative = 3.0;
    private double kTurretToleranceDeg = 1.0;
    private double kPivotToleranceDeg = 1.0;
    private double kFlywheelToleranceRpm = 10.0;
    private double kMinTurretAngle = -110.0;
    private double kMaxTurretAngle = 17.0;

    private double kRpmSlope = 303.0;
    private double kRpmIntercept = 1966.7;
    private double kMaxSafeRpm = 6000.0;
    private double kDragCompensation = 2.0;
    private double kCloseRangeLiftNumerator = 3.0;

    private final double kGravity = 9.81;
    private final double kWheelRadiusMeters = Units.inchesToMeters(2.0) * 0.70;

    private double calculatedAutoAimRpm = 0.0;
    private final LinearFilter rpmFilter = LinearFilter.movingAverage(10);

    private boolean hasCalibratedTurret = false;
    private boolean hasReachedSpeed = false;
    private boolean autoAimEnabled = false;
    private boolean flywheelWithinTolerance = false;

    private boolean testModeEnabled = false;
    private double testManualRpm = 4000.0;

    public Shooter(TurretIO turretIO, PivotIO pivotIO, FlyWheelIO flywheelIO) {
        this.turretIO = turretIO;
        this.pivotIO = pivotIO;
        this.flywheelIO = flywheelIO;

        SmartDashboard.putNumber("Tuning/Shooter/PivotOffset", kPivotEncoderOffset);
        SmartDashboard.putNumber("Tuning/Shooter/MinPivotAngle", kMinPivotAngle);
        SmartDashboard.putNumber("Tuning/Shooter/MaxPivotAngle", kMaxPivotAngle);

        SmartDashboard.putNumber("Tuning/Shooter/ShootRPM", kShootRpm);
        SmartDashboard.putNumber("Tuning/Shooter/FeederShootRPM", kFeederShootRpm);
        SmartDashboard.putNumber("Tuning/Shooter/SpitRPM", kSpitRpm);
        SmartDashboard.putNumber("Tuning/Shooter/FeederSpitRPM", kFeederSpitRpm);

        SmartDashboard.putNumber("Tuning/Shooter/TargetHeightRel", kTargetHeightRelative);
        SmartDashboard.putNumber("Tuning/Shooter/TurretTolerance", kTurretToleranceDeg);
        SmartDashboard.putNumber("Tuning/Shooter/PivotTolerance", kPivotToleranceDeg);
        SmartDashboard.putNumber("Tuning/Shooter/FlywheelTolerance", kFlywheelToleranceRpm);
        SmartDashboard.putNumber("Tuning/Shooter/MinTurretAngle", kMinTurretAngle);
        SmartDashboard.putNumber("Tuning/Shooter/MaxTurretAngle", kMaxTurretAngle);

        SmartDashboard.putNumber("Tuning/Shooter/AutoAim_Slope", kRpmSlope);
        SmartDashboard.putNumber("Tuning/Shooter/AutoAim_Intercept", kRpmIntercept);
        SmartDashboard.putNumber("Tuning/Shooter/AutoAim_MaxSafeRpm", kMaxSafeRpm);
        SmartDashboard.putNumber("Tuning/Shooter/AutoAim_DragComp", kDragCompensation);
        SmartDashboard.putNumber("Tuning/Shooter/AutoAim_CloseRangeLift", kCloseRangeLiftNumerator);

        SmartDashboard.putBoolean("Calibration/TEST_EnableManualRpm", testModeEnabled);
        SmartDashboard.putNumber("Calibration/TEST_ManualRpm", testManualRpm);
        SmartDashboard.putNumber("Calibration/TEST_DistanceToTarget", 0.0);
    }

    public void setupAutoAimReferences(Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.targetSupplier = targetSupplier;
    }

    private double convertRpmToMps(double rpm) {
        return (rpm * 2.0 * Math.PI / 60.0) * kWheelRadiusMeters;
    }

    @Override
    public void periodic() {
        kPivotEncoderOffset = SmartDashboard.getNumber("Tuning/Shooter/PivotOffset", kPivotEncoderOffset);
        kMinPivotAngle = SmartDashboard.getNumber("Tuning/Shooter/MinPivotAngle", kMinPivotAngle);
        kMaxPivotAngle = SmartDashboard.getNumber("Tuning/Shooter/MaxPivotAngle", kMaxPivotAngle);

        kShootRpm = SmartDashboard.getNumber("Tuning/Shooter/ShootRPM", kShootRpm);
        kFeederShootRpm = SmartDashboard.getNumber("Tuning/Shooter/FeederShootRPM", kFeederShootRpm);
        kSpitRpm = SmartDashboard.getNumber("Tuning/Shooter/SpitRPM", kSpitRpm);
        kFeederSpitRpm = SmartDashboard.getNumber("Tuning/Shooter/FeederSpitRPM", kFeederSpitRpm);

        kTargetHeightRelative = SmartDashboard.getNumber("Tuning/Shooter/TargetHeightRel", kTargetHeightRelative);
        kTurretToleranceDeg = SmartDashboard.getNumber("Tuning/Shooter/TurretTolerance", kTurretToleranceDeg);
        kPivotToleranceDeg = SmartDashboard.getNumber("Tuning/Shooter/PivotTolerance", kPivotToleranceDeg);
        kFlywheelToleranceRpm = SmartDashboard.getNumber("Tuning/Shooter/FlywheelTolerance", kFlywheelToleranceRpm);
        kMinTurretAngle = SmartDashboard.getNumber("Tuning/Shooter/MinTurretAngle", kMinTurretAngle);
        kMaxTurretAngle = SmartDashboard.getNumber("Tuning/Shooter/MaxTurretAngle", kMaxTurretAngle);

        kRpmSlope = SmartDashboard.getNumber("Tuning/Shooter/AutoAim_Slope", kRpmSlope);
        kRpmIntercept = SmartDashboard.getNumber("Tuning/Shooter/AutoAim_Intercept", kRpmIntercept);
        kMaxSafeRpm = SmartDashboard.getNumber("Tuning/Shooter/AutoAim_MaxSafeRpm", kMaxSafeRpm);
        kDragCompensation = SmartDashboard.getNumber("Tuning/Shooter/AutoAim_DragComp", kDragCompensation);
        kCloseRangeLiftNumerator = SmartDashboard.getNumber("Tuning/Shooter/AutoAim_CloseRangeLift",
                kCloseRangeLiftNumerator);

        testModeEnabled = SmartDashboard.getBoolean("Calibration/TEST_EnableManualRpm", testModeEnabled);
        testManualRpm = SmartDashboard.getNumber("Calibration/TEST_ManualRpm", testManualRpm);

        turretIO.updateInputs(turretInputs);
        pivotIO.updateInputs(pivotInputs);
        flywheelIO.updateInputs(flywheelInputs);

        Logger.processInputs("Shooter/Turret", turretInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);

        SmartDashboard.putNumber("Turret/CurrentAngle", getTurretPosition());
        SmartDashboard.putNumber("Pivot/CurrentAngle", getPivotPosition());
        SmartDashboard.putBoolean("Turret/InitialSensor", turretInputs.initialLimitHit);
        SmartDashboard.putBoolean("Turret/Max1Sensor", turretInputs.max1LimitHit);
        SmartDashboard.putBoolean("Turret/Max2Sensor", turretInputs.max2LimitHit);

        if (turretInputs.initialLimitHit && !hasCalibratedTurret) {
            hasCalibratedTurret = true;
        }

        if (autoAimEnabled && robotPoseSupplier != null && targetSupplier != null) {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d targetLocation = targetSupplier.get();

            double dist = getDistanceToTarget(robotPose, targetLocation);
            SmartDashboard.putNumber("Calibration/TEST_DistanceToTarget", dist);

            double rawTargetRpm;
            if (testModeEnabled) {
                rawTargetRpm = testManualRpm;
            } else {
                rawTargetRpm = (kRpmSlope * dist) + kRpmIntercept;
            }

            rawTargetRpm = MathUtil.clamp(rawTargetRpm, 0, kMaxSafeRpm);
            double targetRpm = rpmFilter.calculate(rawTargetRpm);

            double dynamicMps = convertRpmToMps(targetRpm);
            double targetPivot = calculatePivotAngleNumeric(dist, dynamicMps);
            double targetTurret = calculateTurretAngle(robotPose, targetLocation);

            setTurretSetpoint(targetTurret);
            setPivotPosition(targetPivot);

            calculatedAutoAimRpm = targetRpm;
        }

        boolean limitHit = turretInputs.max1LimitHit || turretInputs.max2LimitHit;
        SmartDashboard.putBoolean("Turret/LimitHitActive", limitHit);

        if (limitHit) {
            if (getTurretPosition() > 0) {
                if (currentTurretTarget > getTurretPosition()) {
                    currentTurretTarget = getTurretPosition();
                }
            } else {
                if (currentTurretTarget < getTurretPosition()) {
                    currentTurretTarget = getTurretPosition();
                }
            }
        }

        currentTurretTarget = MathUtil.clamp(currentTurretTarget, kMinTurretAngle, kMaxTurretAngle);
        turretIO.runSetpoint(Degrees.of(currentTurretTarget));

        flywheelIO.runVelocity(RPM.of(currentFlywheelTargetRpm));
        flywheelIO.runCentrifugeVelocity(RPM.of(currentFeederTargetRpm));
        flywheelIO.runFeederVelocity(RPM.of(currentFeederTargetRpm));
    }

    public void setFlywheelVelocity(double rpm) {
        this.currentFlywheelTargetRpm = rpm;
    }

    public void runFeeder(double rpm) {
        this.currentFeederTargetRpm = rpm;
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
            if (error <= kFlywheelToleranceRpm) {
                flywheelWithinTolerance = true;
            }
        } else {
            if (error > kFlywheelToleranceRpm * 4.0) {
                flywheelWithinTolerance = false;
            }
        }

        return flywheelWithinTolerance;
    }

    private Command runShootSequence(double manualFlywheelTargetRpm, double feederTargetRpm) {
        return this.run(() -> {
            double activeRpm = autoAimEnabled ? calculatedAutoAimRpm : manualFlywheelTargetRpm;

            setFlywheelVelocity(activeRpm);

            if (!hasReachedSpeed && isFlywheelAtSpeed()) {
                hasReachedSpeed = true;
            }

            if (hasReachedSpeed) {
                runFeeder(feederTargetRpm);
            } else {
                runFeeder(0.0);
            }

        }).beforeStarting(() -> {
            hasReachedSpeed = false;
            flywheelWithinTolerance = false;
        }).finallyDo(() -> {
            hasReachedSpeed = false;
            flywheelWithinTolerance = false;
            stop();
        });
    }

    public Command shootCommand() {
        return runShootSequence(calculatedAutoAimRpm, kFeederShootRpm);
    }

    public Command spitCommand() {
        return runShootSequence(kSpitRpm, kFeederSpitRpm);
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

        if (discriminant < 0) {
            return kMinPivotAngle;
        }

        double idealAngle = Math.toDegrees(Math.atan((v2 - Math.sqrt(discriminant)) / (kGravity * distanceMeters)));

        double dragCompensation = kDragCompensation;
        double closeRangeLift = kCloseRangeLiftNumerator / distanceMeters;

        double finalAngle = idealAngle;

        return finalAngle;
    }

    public void setTurretSetpoint(double degrees) {
        this.currentTurretTarget = degrees;
    }

    public double getTurretPosition() {
        return Units.radiansToDegrees(turretInputs.positionRads);
    }

    public void setPivotPosition(double degreesReal) {
        double clampedDegrees = MathUtil.clamp(degreesReal, kMinPivotAngle, kMaxPivotAngle);
        pivotIO.runSetpoint(Degrees.of(clampedDegrees - kMaxPivotAngle));
    }

    public double getPivotPosition() {
        return kMaxPivotAngle + Units.radiansToDegrees(pivotInputs.positionRads);
    }

    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
    }

    public void stop() {
        currentFlywheelTargetRpm = 0.0;
        currentFeederTargetRpm = 0.0;
    }

    public void resetTurretEncoder() {
        turretIO.resetEncoder();
        hasCalibratedTurret = false;
    }

    public void resetPivotEncoder() {
        pivotIO.resetEncoder();
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }
}