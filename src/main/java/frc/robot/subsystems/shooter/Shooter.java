package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier; // <- ADICIONADO PARA FUNCIONAR

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

    // VARIÁVEIS ADICIONADAS PARA O AUTO AIM FUNCIONAR
    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Translation2d> targetSupplier;

    private double currentTurretTarget = 0.0;
    private double currentFlywheelTargetRpm = 0.0;
    private Translation2d lastTargetLocation = null;

    private final double kMinPivotAngle = 0.0;
    private final double kMaxPivotAngle = 65.0;

    private double kPivotEncoderOffset = 35.0;

    private double kShotVelocityMPS = 18.0;
    private double kShootRpm = 4500.0;
    private double kFeederShootRpm = 3000.0;
    private double kSpitRpm = 1000.0;
    private double kFeederSpitRpm = 2000.0;

    private final double kGravity = 9.81;
    private final double kTargetHeightRelative = 1.8;

    private final double kTurretToleranceDeg = 2.0;
    private final double kPivotToleranceDeg = 2.0;
    private final double kFlywheelToleranceRpm = 100.0;

    private final double kMinTurretAngle = -110.0;
    private final double kMaxTurretAngle = 35.0;
    private boolean hasCalibratedTurret = false;

    private boolean hasReachedSpeed = false;

    private boolean autoAimEnabled = false;

    public Shooter(TurretIO turretIO, PivotIO pivotIO, FlyWheelIO flywheelIO) {
        this.turretIO = turretIO;
        this.pivotIO = pivotIO;
        this.flywheelIO = flywheelIO;

        SmartDashboard.putNumber("Tuning/Shooter/PivotOffset", kPivotEncoderOffset);
        SmartDashboard.putNumber("Tuning/Shooter/ShotVelocityMPS", kShotVelocityMPS);
        SmartDashboard.putNumber("Tuning/Shooter/ShootRPM", kShootRpm);
        SmartDashboard.putNumber("Tuning/Shooter/FeederShootRPM", kFeederShootRpm);
        SmartDashboard.putNumber("Tuning/Shooter/SpitRPM", kSpitRpm);
        SmartDashboard.putNumber("Tuning/Shooter/FeederSpitRPM", kFeederSpitRpm);
    }

    // MÉTODO NOVO PARA RECEBER A POSIÇÃO DO ROBÔ DO ROBOTCONTAINER
    public void setupAutoAimReferences(Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.targetSupplier = targetSupplier;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Tuning/Shooter/PivotOffset", kPivotEncoderOffset);

        kShotVelocityMPS = SmartDashboard.getNumber("Tuning/Shooter/ShotVelocityMPS", kShotVelocityMPS);
        kShootRpm = SmartDashboard.getNumber("Tuning/Shooter/ShootRPM", kShootRpm);
        kFeederShootRpm = SmartDashboard.getNumber("Tuning/Shooter/FeederShootRPM", kFeederShootRpm);
        kSpitRpm = SmartDashboard.getNumber("Tuning/Shooter/SpitRPM", kSpitRpm);
        kFeederSpitRpm = SmartDashboard.getNumber("Tuning/Shooter/FeederSpitRPM", kFeederSpitRpm);

        SmartDashboard.putNumber("Pivot/CurrentAngle", getPivotPosition());
        SmartDashboard.putNumber("Pivot/RawMotorRotations", pivotInputs.positionRads / (2 * Math.PI));

        turretIO.updateInputs(turretInputs);
        pivotIO.updateInputs(pivotInputs);
        flywheelIO.updateInputs(flywheelInputs);

        Logger.processInputs("Shooter/Turret", turretInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);

        if (turretInputs.initialLimitHit && !hasCalibratedTurret) {
            turretIO.setEncoderPosition(Degrees.of(30.0));
            hasCalibratedTurret = true;
        }

        boolean limitHit = turretInputs.max1LimitHit && turretInputs.max2LimitHit;
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

        // ==========================================
        // CÓDIGO CORRIGIDO DA MIRA AUTOMÁTICA AQUI
        // ==========================================
        if (autoAimEnabled && robotPoseSupplier != null && targetSupplier != null) {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d targetLocation = targetSupplier.get();

            double dist = getDistanceToTarget(robotPose, targetLocation);
            double targetPivot = calculatePivotAngleNumeric(dist);
            double targetTurret = calculateTurretAngle(robotPose, targetLocation);

            setTurretSetpoint(targetTurret);
            setPivotPosition(targetPivot);
        }
        // ==========================================

        currentTurretTarget = MathUtil.clamp(currentTurretTarget, kMinTurretAngle, kMaxTurretAngle);

        SmartDashboard.putBoolean("Turret/InitialSensor", turretInputs.initialLimitHit);
        SmartDashboard.putBoolean("Turret/Max1Sensor", turretInputs.max1LimitHit);
        SmartDashboard.putBoolean("Turret/Max2Sensor", turretInputs.max2LimitHit);
        SmartDashboard.putBoolean("Turret/IsCalibrated", hasCalibratedTurret);
        SmartDashboard.putBoolean("Turret/LimitHitActive", limitHit);
        SmartDashboard.putNumber("Turret/CurrentAngle", getTurretPosition());
        SmartDashboard.putBoolean("Shooter/AutoAimActive", autoAimEnabled);

        turretIO.runSetpoint(Degrees.of(currentTurretTarget));

        if (currentFlywheelTargetRpm < 10) {
            flywheelIO.stop();
        } else {
            flywheelIO.runVelocity(RPM.of(currentFlywheelTargetRpm));
        }

        Logger.recordOutput("Viz/Shooter/TurretRotation3d", new Rotation3d(0, 0, turretInputs.positionRads));
        Logger.recordOutput("Viz/Shooter/PivotRotation3d", new Rotation3d(0, -pivotInputs.positionRads, 0));

        if (lastTargetLocation != null) {
            Logger.recordOutput("Viz/Shooter/CurrentTarget",
                    new Pose3d(lastTargetLocation.getX(), lastTargetLocation.getY(), 2.0, new Rotation3d()));
        }

        Logger.recordOutput("Shooter/TargetRPM", currentFlywheelTargetRpm);
        Logger.recordOutput("Shooter/CurrentRPM", getFlywheelRpm());
        Logger.recordOutput("Shooter/IsAtSpeed", isFlywheelAtSpeed());
    }

    public void setFlywheelVelocity(double rpm) {
        this.currentFlywheelTargetRpm = rpm;
    }

    public void runFeeder(double rpm) {
        flywheelIO.runCentrifugeVelocity(RPM.of(rpm));
        flywheelIO.runFeederVelocity(RPM.of(rpm));
        Logger.recordOutput("Shooter/FeederRequestRpm", rpm);
    }

    public double getFlywheelRpm() {
        return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
    }

    public boolean isFlywheelAtSpeed() {
        double error = Math.abs(getFlywheelRpm() - currentFlywheelTargetRpm);
        return currentFlywheelTargetRpm > 100 && error <= kFlywheelToleranceRpm;
    }

    private Command runShootSequence(double flywheelTargetRpm, double feederTargetRpm) {
        return this.run(() -> {
            setFlywheelVelocity(flywheelTargetRpm);
            if (isFlywheelAtSpeed()) {
                hasReachedSpeed = true;
            }
            if (hasReachedSpeed) {
                runFeeder(feederTargetRpm);
            } else {
                runFeeder(0.0);
            }
        })
                .beforeStarting(() -> {
                    hasReachedSpeed = false;
                })
                .finallyDo(() -> {
                    hasReachedSpeed = false;
                    stop();
                });
    }

    public Command shootCommand() {
        return runShootSequence(kShootRpm, kFeederShootRpm);
    }

    public Command spitCommand() {
        return runShootSequence(kSpitRpm, kFeederSpitRpm);
    }

    public boolean isReadyToShoot(double targetPivotAngle, double targetTurretAngle) {
        double turretError = Math.abs(getTurretPosition() - targetTurretAngle);
        boolean turretReady = turretError < kTurretToleranceDeg;

        double pivotError = Math.abs(getPivotPosition() - targetPivotAngle);
        boolean pivotReady = pivotError < kPivotToleranceDeg;

        return turretReady && pivotReady && isFlywheelAtSpeed();
    }

    public double getDistanceToTarget(Pose2d robotPose, Translation2d targetLocation) {
        return robotPose.getTranslation().getDistance(targetLocation);
    }

    public double calculateTurretAngle(Pose2d robotPose, Translation2d targetLocation) {
        this.lastTargetLocation = targetLocation;
        double dx = targetLocation.getX() - robotPose.getX();
        double dy = targetLocation.getY() - robotPose.getY();
        double angleRad = Math.atan2(dy, dx);
        double angleDeg = Math.toDegrees(angleRad);
        double robotHeading = robotPose.getRotation().getDegrees();
        return MathUtil.inputModulus(robotHeading - angleDeg, -180, 180);
    }

    public double calculatePivotAngleNumeric(double distanceMeters) {
        double v2 = kShotVelocityMPS * kShotVelocityMPS;
        double v4 = v2 * v2;
        double discriminant = v4
                - kGravity * (kGravity * distanceMeters * distanceMeters + 2 * kTargetHeightRelative * v2);
        if (discriminant < 0)
            return 45.0;
        double numerator = v2 - Math.sqrt(discriminant);
        double denominator = kGravity * distanceMeters;
        return Math.toDegrees(Math.atan(numerator / denominator));
    }

    public void setTurretSetpoint(double degrees) {
        this.currentTurretTarget = degrees;
    }

    public double getTurretPosition() {
        return Units.radiansToDegrees(turretInputs.positionRads);
    }

    public void setPivotPosition(double degreesReal) {
        double clampedDegrees = MathUtil.clamp(degreesReal, kMinPivotAngle, kMaxPivotAngle);
        double motorSetpoint = clampedDegrees - 65.0;

        pivotIO.runSetpoint(edu.wpi.first.units.Units.Degrees.of(motorSetpoint));
        Logger.recordOutput("Shooter/Pivot/SetpointReal", clampedDegrees);
    }

    public double getPivotPosition() {
        return 65.0 + Units.radiansToDegrees(pivotInputs.positionRads);
    }

    public void resetTurretEncoder() {
        turretIO.resetEncoder();
        hasCalibratedTurret = false;
    }

    public void resetPivotEncoder() {
        pivotIO.resetEncoder();
    }

    public Command testFeederCommand() {
        return this.run(() -> {
            runFeeder(2000);
            Logger.recordOutput("Shooter/TestMode", "Force Feeder ON");
        })
                .finallyDo(() -> {
                    runFeeder(0);
                    Logger.recordOutput("Shooter/TestMode", "Force Feeder OFF");
                });
    }

    public boolean isAlignedToTarget(Pose2d robotPose, Translation2d targetLocation) {
        double targetTurretAngle = calculateTurretAngle(robotPose, targetLocation);
        double safeTurretSetpoint = MathUtil.clamp(targetTurretAngle, kMinTurretAngle, kMaxTurretAngle);

        double distToTarget = getDistanceToTarget(robotPose, targetLocation);
        double targetPivotAngle = calculatePivotAngleNumeric(distToTarget);
        if (Double.isNaN(targetPivotAngle)) {
            targetPivotAngle = 45.0;
        }

        double turretError = Math.abs(getTurretPosition() - safeTurretSetpoint);
        double pivotError = Math.abs(getPivotPosition() - targetPivotAngle);

        return (turretError < kTurretToleranceDeg) && (pivotError < kPivotToleranceDeg);
    }

    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
    }

public void stop() {
        currentFlywheelTargetRpm = 0.0;
        flywheelIO.stop();
        runFeeder(0);
    }
}
