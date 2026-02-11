package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

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

    private double currentTurretTarget = 0.0;
    private double currentFlywheelTargetRpm = 0.0;
    private Translation2d lastTargetLocation = null;

    private final double kMinPivotAngle = 0.0;
    private final double kMaxPivotAngle = 80.0;
    private final double kPivotEncoderOffset = 35.0;

    private final double kShotVelocityMPS = 18.0;
    private final double kGravity = 9.81;
    private final double kTargetHeightRelative = 1.8;

    private final double kTurretToleranceDeg = 2.0;
    private final double kPivotToleranceDeg = 2.0;
    private final double kFlywheelToleranceRpm = 500.0;

    private final double kShootRpm = 3500.0;
    private final double kFeederShootRpm = 3000.0;
    private final double kSpitRpm = 1000.0;
    private final double kFeederSpitRpm = 1000.0;

    public Shooter(TurretIO turretIO, PivotIO pivotIO, FlyWheelIO flywheelIO) {
        this.turretIO = turretIO;
        this.pivotIO = pivotIO;
        this.flywheelIO = flywheelIO;
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);
        pivotIO.updateInputs(pivotInputs);
        flywheelIO.updateInputs(flywheelInputs);

        Logger.processInputs("Shooter/Turret", turretInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);

        turretIO.runSetpoint(edu.wpi.first.units.Units.Degrees.of(currentTurretTarget));

        if (currentFlywheelTargetRpm < 10) {
            flywheelIO.stop();
        } else {
            flywheelIO.runVelocity(edu.wpi.first.units.Units.RPM.of(currentFlywheelTargetRpm));
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
        flywheelIO.runCentrifugeVelocity(edu.wpi.first.units.Units.RPM.of(rpm));
        flywheelIO.runFeederVelocity(edu.wpi.first.units.Units.RPM.of(rpm));
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
                runFeeder(feederTargetRpm);
            } else {
                runFeeder(0.0);
            }
        })
        .finallyDo(() -> {
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
        return MathUtil.inputModulus(angleDeg - robotHeading, -180, 180);
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
        double motorSetpoint = clampedDegrees - kPivotEncoderOffset;
        pivotIO.runSetpoint(edu.wpi.first.units.Units.Degrees.of(motorSetpoint));
        Logger.recordOutput("Shooter/Pivot/SetpointReal", clampedDegrees);
    }

    public double getPivotPosition() {
        return Units.radiansToDegrees(pivotInputs.positionRads) + kPivotEncoderOffset;
    }

    public void resetTurretEncoder() {
        turretIO.resetEncoder();
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

    public void stop() {
        currentFlywheelTargetRpm = 0.0;
        turretIO.stop();
        pivotIO.stop();
        flywheelIO.stop(); 
        runFeeder(0);
    }
}