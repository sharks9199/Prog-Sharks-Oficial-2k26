package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
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
    private Translation2d lastTargetLocation = null;

    private final double kMinPivotAngle = 50.0;
    private final double kMaxPivotAngle = 68.89;
    private double kPivotEncoderOffset = 35.0;

    private double kShootRpm = 4000.0;
    private double kFeederShootRpm = 4000.0;
    private double kSpitRpm = 1000.0;
    private double kFeederSpitRpm = 2000.0;

    private final double kGravity = 9.81;
    private final double kTargetHeightRelative = 1.8;
    private final double kTurretToleranceDeg = 1.0;
    private final double kPivotToleranceDeg = 1.0;
    private final double kFlywheelToleranceRpm = 50.0;
    private final double kMinTurretAngle = -110.0;
    private final double kMaxTurretAngle = 17.0;
    private double calculatedAutoAimRpm = 0.0;

    // Regressão Linear
    private double kRpmSlope = 500.0;
    private double kRpmIntercept = 2500.0;
    private final double kMaxSafeRpm = 6000.0;
    private final double kWheelRadiusMeters = Units.inchesToMeters(0.4);

    // ==========================================
    // FILTROS (Suavização de ruído da Câmera/Odometria)
    // ==========================================
    // Média móvel das últimas 10 leituras (aprox 0.2s) para estabilizar os Krakens
    private final LinearFilter rpmFilter = LinearFilter.movingAverage(10);

    private boolean hasCalibratedTurret = false;
    private boolean hasReachedSpeed = false;
    private boolean autoAimEnabled = false;

    // ==========================================
    // INÍCIO DO BLOCO DE TESTES PARA CALIBRAÇÃO (FÁCIL DE APAGAR)
    // ==========================================
    private boolean testModeEnabled = false;
    private double testManualRpm = 4000.0;
    // ==========================================
    // FIM DO BLOCO DE TESTES
    // ==========================================

    public Shooter(TurretIO turretIO, PivotIO pivotIO, FlyWheelIO flywheelIO) {
        this.turretIO = turretIO;
        this.pivotIO = pivotIO;
        this.flywheelIO = flywheelIO;

        SmartDashboard.putNumber("Tuning/Shooter/PivotOffset", kPivotEncoderOffset);
        SmartDashboard.putNumber("Tuning/Shooter/ShootRPM", kShootRpm);
        SmartDashboard.putNumber("Tuning/Shooter/FeederShootRPM", kFeederShootRpm);

        SmartDashboard.putNumber("Tuning/Shooter/AutoAim_Slope", kRpmSlope);
        SmartDashboard.putNumber("Tuning/Shooter/AutoAim_Intercept", kRpmIntercept);

        // ==========================================
        // BLOCO DE TESTES NO DASHBOARD
        // ==========================================
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
        kShootRpm = SmartDashboard.getNumber("Tuning/Shooter/ShootRPM", kShootRpm);
        kRpmSlope = SmartDashboard.getNumber("Tuning/Shooter/AutoAim_Slope", kRpmSlope);
        kRpmIntercept = SmartDashboard.getNumber("Tuning/Shooter/AutoAim_Intercept", kRpmIntercept);

        // ==========================================
        // INÍCIO DA ATUALIZAÇÃO DO BLOCO DE TESTES
        // ==========================================
        testModeEnabled = SmartDashboard.getBoolean("Calibration/TEST_EnableManualRpm", testModeEnabled);
        testManualRpm = SmartDashboard.getNumber("Calibration/TEST_ManualRpm", testManualRpm);
        // ==========================================
        // FIM DA ATUALIZAÇÃO DO BLOCO DE TESTES
        // ==========================================

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

        // ==========================================
        // LÓGICA DO AUTO AIM COM FILTRO
        // ==========================================
        if (autoAimEnabled && robotPoseSupplier != null && targetSupplier != null) {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d targetLocation = targetSupplier.get();

            double dist = getDistanceToTarget(robotPose, targetLocation);
            SmartDashboard.putNumber("Calibration/TEST_DistanceToTarget", dist);

            // 1. Calcula o RPM bruto com base na distância
            double rawTargetRpm = (kRpmSlope * dist) + kRpmIntercept;
            rawTargetRpm = MathUtil.clamp(rawTargetRpm, 0, kMaxSafeRpm);

            // 2. Aplica o filtro para ignorar micro-vibrações da odometria
            double targetRpm = rpmFilter.calculate(rawTargetRpm);

            // 3. Usa o RPM filtrado para o restante dos cálculos
            double dynamicMps = convertRpmToMps(targetRpm);
            double targetPivot = calculatePivotAngleNumeric(dist, dynamicMps);
            double targetTurret = calculateTurretAngle(robotPose, targetLocation);

            setTurretSetpoint(targetTurret);
            setPivotPosition(targetPivot);
            calculatedAutoAimRpm = targetRpm;
        }

        // ==========================================
        // SEGURANÇA DE FIM DE CURSO
        // ==========================================
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

        // Controle do Flywheel
        if (currentFlywheelTargetRpm < 10)
            flywheelIO.stop();
        else
            flywheelIO.runVelocity(RPM.of(currentFlywheelTargetRpm));
    }

    public void setFlywheelVelocity(double rpm) {
        this.currentFlywheelTargetRpm = rpm;
    }

    public void runFeeder(double rpm) {
        flywheelIO.runCentrifugeVelocity(RPM.of(rpm));
        flywheelIO.runFeederVelocity(RPM.of(rpm));
    }

    public double getFlywheelRpm() {
        return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
    }

    public boolean isFlywheelAtSpeed() {
        return currentFlywheelTargetRpm > 100
                && Math.abs(getFlywheelRpm() - currentFlywheelTargetRpm) <= kFlywheelToleranceRpm;
    }

    private Command runShootSequence(double manualFlywheelTargetRpm, double feederTargetRpm) {
        return this.run(() -> {
            // 1. Define o RPM base (se autoAim tá ligado, usa o calculado filtrado, senão
            // usa o manual)
            double activeRpm = autoAimEnabled ? calculatedAutoAimRpm : manualFlywheelTargetRpm;

            // 2. MODO DE TESTE ABSOLUTO
            if (testModeEnabled) {
                activeRpm = testManualRpm;
            }

            setFlywheelVelocity(activeRpm);

            if (isFlywheelAtSpeed())
                hasReachedSpeed = true;

            if (hasReachedSpeed)
                runFeeder(feederTargetRpm);
            else
                runFeeder(0.0);

        }).beforeStarting(() -> hasReachedSpeed = false).finallyDo(() -> {
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
            return kMinPivotAngle;
        return Math.toDegrees(Math.atan((v2 - Math.sqrt(discriminant)) / (kGravity * distanceMeters)));
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
        flywheelIO.stop();
        runFeeder(0);
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