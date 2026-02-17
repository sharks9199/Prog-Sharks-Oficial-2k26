package frc.robot.commands.Autos;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // <-- Import adicionado
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FieldConstants.FieldPoses;
import frc.robot.subsystems.shooter.Shooter;

public class AutoAim extends Command {

    private final Shooter shooter;
    private final Supplier<Pose2d> robotPoseSupplier;
    private Translation2d targetLocation;

    private final double kMinTurretAngle = -110.0;
    private final double kMaxTurretAngle = 35.0;

    private final double kShootingRPM = 3500.0; 
    private final double kFeederVolts = 10.0;
    

    public AutoAim(Shooter shooter, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter);
        SmartDashboard.putBoolean("ReadyToScore", false);
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetLocation = FieldPoses.kHubRed;
        } else {
            targetLocation = FieldPoses.kHubBlue;
        }
    }

    @Override
    public void execute() {
        Pose2d currentRobotPose = robotPoseSupplier.get();

        double targetTurretAngle = shooter.calculateTurretAngle(currentRobotPose, targetLocation);
        double safeTurretSetpoint = MathUtil.clamp(targetTurretAngle, kMinTurretAngle, kMaxTurretAngle);
        
        // Pivot
        double distToTarget = shooter.getDistanceToTarget(currentRobotPose, targetLocation);
        double targetPivotAngle = shooter.calculatePivotAngleNumeric(distToTarget);

        shooter.setTurretSetpoint(safeTurretSetpoint);
        double finalPivotSetpoint = 45.0;
        if (!Double.isNaN(targetPivotAngle)) {
            finalPivotSetpoint = targetPivotAngle;
        }
        shooter.setPivotPosition(finalPivotSetpoint);

        shooter.setFlywheelVelocity(kShootingRPM);

        boolean readyToShoot = shooter.isReadyToShoot(finalPivotSetpoint, safeTurretSetpoint);

        SmartDashboard.putBoolean("ReadyToScore", readyToShoot);

        if (readyToShoot) {
            shooter.runFeeder(kFeederVolts);
            Logger.recordOutput("AutoAim/Status", "ATIRANDO!");
        } else {
            shooter.runFeeder(0.0);
            Logger.recordOutput("AutoAim/Status", "Mirando/Acelerando...");
        }

        Logger.recordOutput("AutoAim/TargetDist", distToTarget);
        Logger.recordOutput("AutoAim/CalculatedPivot", targetPivotAngle);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop(); 
        shooter.setFlywheelVelocity(0);
        shooter.runFeeder(0);

        SmartDashboard.putBoolean("ReadyToScore", false);
    }
}