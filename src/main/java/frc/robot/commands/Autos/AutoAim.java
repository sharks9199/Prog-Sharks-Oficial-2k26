package frc.robot.commands.Autos;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FieldConstants.FieldPoses;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Comando de AutoAim que calcula Turret e Pivot em tempo real
 * considerando a saída física de 35 graus do Shooter.
 */
public class AutoAim extends Command {

    private final Shooter shooter;
    private final Supplier<Pose2d> robotPoseSupplier;
    private Translation2d targetLocation;

    // Limites da Turret (conforme seu Shooter.java)
    private final double kMinTurretAngle = -110.0;
    private final double kMaxTurretAngle = 35.0;

    // Configurações de Tiro
    private final double kShootingRPM = 4500.0; // Sincronizado com kShootRpm do shooter
    private final double kFeederRPM = 3000.0;   // RPM para o feeder alimentar a nota

    public AutoAim(Shooter shooter, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetLocation = FieldPoses.kHubRed;
        } else {
            targetLocation = FieldPoses.kHubBlue;
        }
        
        Logger.recordOutput("AutoAim/Active", true);
        SmartDashboard.putBoolean("ReadyToScore", false);
    }

    @Override
    public void execute() {
        Pose2d currentRobotPose = robotPoseSupplier.get();

        double targetTurretAngle = shooter.calculateTurretAngle(currentRobotPose, targetLocation);
        double safeTurretSetpoint = MathUtil.clamp(targetTurretAngle, kMinTurretAngle, kMaxTurretAngle);
        
        double distToTarget = shooter.getDistanceToTarget(currentRobotPose, targetLocation);
        double targetPivotAngle = shooter.calculatePivotAngleNumeric(distToTarget);

        shooter.setTurretSetpoint(safeTurretSetpoint);
        double finalPivotSetpoint = 50.0; 
        if (!Double.isNaN(targetPivotAngle)) {
            finalPivotSetpoint = targetPivotAngle;
        }
        shooter.setPivotPosition(finalPivotSetpoint);

        //shooter.setFlywheelVelocity(kShootingRPM);

        boolean readyToShoot = shooter.isReadyToShoot(finalPivotSetpoint, safeTurretSetpoint);
        SmartDashboard.putBoolean("ReadyToScore", readyToShoot);

        Logger.recordOutput("AutoAim/TargetDist", distToTarget);
        Logger.recordOutput("AutoAim/CalculatedPivotAngle", targetPivotAngle);
        Logger.recordOutput("AutoAim/FinalPivotSetpoint", finalPivotSetpoint);
        Logger.recordOutput("AutoAim/TargetTurretAngle", targetTurretAngle);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop(); 
        Logger.recordOutput("AutoAim/Active", false);
        Logger.recordOutput("AutoAim/Status", "Parado");
        SmartDashboard.putBoolean("ReadyToScore", false);
    }
}