package frc.robot.commands.Autos;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;

public class SmartTrench {

    public static Command run(Drive drive) {
        return Commands.defer(() -> {

            Pose2d currentPose = drive.getPose();
            double fieldCenterY = 4.05;

            var allianceOpt = DriverStation.getAlliance();
            boolean isRed = allianceOpt.isPresent() && allianceOpt.get() == Alliance.Red;
            double cosHeading = Math.cos(currentPose.getRotation().getRadians());
            boolean isFacingForward = isRed ? (cosHeading < 0) : (cosHeading > 0);
            boolean useLeftTrenchLogic = calculateIsLeft(currentPose.getY(), fieldCenterY);

            PathPlannerPath selectedPath = FieldConstants.FieldPoses.getAutoTrenchPath(
                    currentPose,
                    useLeftTrenchLogic,
                    isFacingForward);

            if (selectedPath != null) {
                Pose2d originalStart = selectedPath.getStartingHolonomicPose().orElse(currentPose);
                Rotation2d bestRotation = isFacingForward ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
                Pose2d smartStartPose = new Pose2d(originalStart.getTranslation(), bestRotation);
                double goalVelocity = 0.7;

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