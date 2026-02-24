package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import frc.robot.subsystems.shooter.Pivot.PivotIOComp;
import frc.robot.subsystems.shooter.Shooter;

public class SmartTrench {
   
    public static Command run(Drive drive) {
        return Commands.defer(() -> {
            
            Pose2d currentPose = drive.getPose();
            double fieldCenterY = 4.05;

            boolean isFacingForward = Math.cos(currentPose.getRotation().getRadians()) > 0;
            boolean useLeftTrenchLogic = calculateIsLeft(currentPose.getY(), fieldCenterY);
            PathPlannerPath selectedPath = FieldConstants.FieldPoses.getAutoTrenchPath(
                    currentPose,
                    useLeftTrenchLogic,
                    isFacingForward);

            if (selectedPath != null) {
                Pose2d smartStartPose = selectedPath.getStartingHolonomicPose().orElse(currentPose);
                double goalVelocity = 0.2;

            //pivot.runSetpoint(Degrees.of(Shooter.kMinPivotAngle));

                Command pathfind = AutoBuilder.pathfindToPose(
                        smartStartPose,
                        AutoConstants.constraints,
                        goalVelocity);

                Command follow = AutoBuilder.followPath(selectedPath);

                return pathfind.andThen(follow);

            } else {
                return Commands.none();
            }

        }, Set.of(drive));
    }

    private static boolean calculateIsLeft(double robotY, double splitY) {
        var allianceOpt = DriverStation.getAlliance();

        if (allianceOpt.isPresent() && allianceOpt.get() == Alliance.Red) {
            return robotY <= splitY;
        } else {
            return robotY > splitY;
        }
    }
}
