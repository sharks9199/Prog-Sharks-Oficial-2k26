package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d; 
import edu.wpi.first.math.geometry.Rotation3d; 
import edu.wpi.first.math.geometry.Translation2d;
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

    // --- SUBSYSTEMS IO ---
    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    
    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    
    private final FlyWheelIO flywheelIO;
    private final FlyWheelIOInputsAutoLogged flywheelInputs = new FlyWheelIOInputsAutoLogged();

    // --- STATE VARIABLES ---
    private double currentTurretTarget = 0.0;
    private double currentFlywheelTargetRpm = 0.0;
    private Translation2d lastTargetLocation = null;

    // --- CONSTANTES ---
    private final double kMinPivotAngle = 0.0;
    private final double kMaxPivotAngle = 80.0;
    private final double kPivotEncoderOffset = 35.0;
    private final double kShotVelocityMPS = 18.0;
    private final double kGravityHalf = 4.95;
    private final double kTargetHeightRelative = 1.8;

    // --- TOLERÂNCIAS PARA ATIRAR (AutoAim) ---
    private final double kTurretToleranceDeg = 2.0;
    private final double kPivotToleranceDeg = 2.0;
    private final double kFlywheelToleranceRpm = 150.0;

    // --- CONSTRUTOR CORRIGIDO ---
    public Shooter(TurretIO turretIO, PivotIO pivotIO, FlyWheelIO flywheelIO) {
        this.turretIO = turretIO;
        this.pivotIO = pivotIO;
        this.flywheelIO = flywheelIO;
    }

    @Override
    public void periodic() {
        // 1. Atualiza Inputs
        turretIO.updateInputs(turretInputs);
        pivotIO.updateInputs(pivotInputs);
        flywheelIO.updateInputs(flywheelInputs);

        // 2. Processa Logs
        Logger.processInputs("Shooter/Turret", turretInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);

        // 3. Roda os Control Loops
        turretIO.runSetpoint(edu.wpi.first.units.Units.Degrees.of(currentTurretTarget));
        
        // --- CORREÇÃO: Manda o Flywheel girar ---
        flywheelIO.runVelocity(edu.wpi.first.units.Units.RPM.of(currentFlywheelTargetRpm));

        // 4. Logs de Visualização 3D
        Logger.recordOutput("Viz/Shooter/TurretRotation3d",
                new Rotation3d(0, 0, turretInputs.positionRads));

        Logger.recordOutput("Viz/Shooter/PivotRotation3d",
                new Rotation3d(0, -pivotInputs.positionRads, 0));

        if (lastTargetLocation != null) {
            Logger.recordOutput("Viz/Shooter/CurrentTarget",
                    new Pose3d(lastTargetLocation.getX(), lastTargetLocation.getY(), 2.0, new Rotation3d()));
        }
    }

    // =================================================================
    // LÓGICA DE DISPARO E FEEDER (Adicionado para AutoAim)
    // =================================================================

    /**
     * Define a velocidade do Flywheel.
     */
    public void setFlywheelVelocity(double rpm) {
        this.currentFlywheelTargetRpm = rpm;
    }

    public void runFeeder(double volts) {
        // Exemplo: feederMotor.setVoltage(volts);
        Logger.recordOutput("Shooter/FeederRequestVolts", volts);
    }

    /**
     * Verifica se Turret, Pivot e Flywheel estão prontos para o disparo.
     */
    public boolean isReadyToShoot(double targetPivotAngle, double targetTurretAngle) {
        // 1. Erro da Torre
        double turretError = Math.abs(getTurretPosition() - targetTurretAngle);
        boolean turretReady = turretError < kTurretToleranceDeg;

        // 2. Erro do Pivot
        double pivotError = Math.abs(getPivotPosition() - targetPivotAngle);
        boolean pivotReady = pivotError < kPivotToleranceDeg;

        // 3. Erro do Flywheel (Só checa se o alvo for > 0)
        double currentRpm = Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
        double flywheelError = Math.abs(currentRpm - currentFlywheelTargetRpm);
        boolean flywheelReady = (currentFlywheelTargetRpm < 100) || (flywheelError < kFlywheelToleranceRpm);

        Logger.recordOutput("Shooter/Ready/Turret", turretReady);
        Logger.recordOutput("Shooter/Ready/Pivot", pivotReady);
        Logger.recordOutput("Shooter/Ready/Flywheel", flywheelReady);

        return turretReady && pivotReady && flywheelReady;
    }

    // =================================================================
    // TURRET
    // =================================================================

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
        double targetRelativeToNose = angleDeg - robotHeading;

        double invertedAngle = -1.0 * targetRelativeToNose;
        double hardwareOffset = -90.0;
        double finalSetpoint = invertedAngle + hardwareOffset;

        return MathUtil.inputModulus(finalSetpoint, -180.0, 180.0);
    }

    // =================================================================
    // PIVOT & MATEMÁTICA
    // =================================================================

    public double calculatePivotAngleNumeric(double distanceMeters) {
        double angleRad = solveProjectileEquation(distanceMeters, kShotVelocityMPS);
        if (Double.isNaN(angleRad)) {
            return Double.NaN;
        }
        return Math.toDegrees(angleRad);
    }

    private double solveProjectileEquation(double d, double v) {
        double v2 = v * v;
        double currentTheta = Math.toRadians(40.0);
        double tolerance = 0.01;
        int maxIterations = 10;

        for (int i = 0; i < maxIterations; i++) {
            double sin2Theta = Math.sin(2 * currentTheta);
            double cos2Theta = Math.cos(2 * currentTheta);
            double cosSquared = Math.pow(Math.cos(currentTheta), 2);

            double f_theta = -(kGravityHalf * d * d) + (d * v2 * sin2Theta / 2.0) - (kTargetHeightRelative * v2 * cosSquared);

            if (Math.abs(f_theta) < tolerance) return currentTheta;

            double f_prime = (d * v2 * cos2Theta) + (kTargetHeightRelative * v2 * sin2Theta);

            if (Math.abs(f_prime) < 1e-6) break;

            currentTheta = currentTheta - (f_theta / f_prime);
        }

        if (currentTheta > 0 && currentTheta < Math.PI / 2) {
            return currentTheta;
        } else {
            return Double.NaN;
        }
    }

    public void setTurretSetpoint(double degrees) {
        this.currentTurretTarget = degrees;
    }

    public void addTurretSetpoint(double deltaDegrees) {
        this.currentTurretTarget += deltaDegrees;
    }

    public double getTurretPosition() {
        return Units.radiansToDegrees(turretInputs.positionRads);
    }

    public void setPivotPosition(double degreesReal) {
        // 1. Clamp de Segurança
        double clampedDegrees = MathUtil.clamp(degreesReal, kMinPivotAngle, kMaxPivotAngle);

        System.out.println("PIVOT ALVO >> Real: " + degreesReal + " | Clamp: " + clampedDegrees);
        Logger.recordOutput("Shooter/Pivot/SetpointRealDegrees", clampedDegrees);

        // 2. Tira o Offset para o Motor
        double motorSetpoint = clampedDegrees - kPivotEncoderOffset;

        Logger.recordOutput("Shooter/Pivot/SetpointMotorRaw", motorSetpoint);

        // 3. Manda pro motor
        pivotIO.runSetpoint(edu.wpi.first.units.Units.Degrees.of(motorSetpoint));
    }

    public double getPivotPosition() {
        double encoderDegrees = Units.radiansToDegrees(pivotInputs.positionRads);
        // 4. Soma o Offset para ler o ângulo real
        return encoderDegrees + kPivotEncoderOffset;
    }

    public double getPivotVolts() {
        return pivotInputs.appliedVolts;
    }

    public double getPivotAmps() {
        return pivotInputs.supplyCurrentAmps;
    }

    public void resetTurretEncoder() {
        turretIO.resetEncoder();
    }

    public void resetPivotEnconder() {
        pivotIO.resetEncoder();
    }

    public void stop() {
        turretIO.stop();
        pivotIO.stop();
        flywheelIO.stop();
        runFeeder(0);
    }
}