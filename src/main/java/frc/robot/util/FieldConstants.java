package frc.robot.util;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final class FieldPoses {

        public static final Optional<Alliance> alliance = DriverStation.getAlliance();

        // TRENCHS
        // Posições Trench BLUE ALLIANCE
        public static final Pose2d kInitBlueTrenchLeft = new Pose2d(3.200, 7.400, new Rotation2d(0));
        public static final Pose2d kFinalBlueTrenchLeft = new Pose2d(5.750, 7.400, new Rotation2d(0));
        public static final Pose2d kInitBlueTrechRight = new Pose2d(3.200, 0.700, new Rotation2d(0));
        public static final Pose2d kFinalBlueTrenchRight = new Pose2d(5.750, 0.700, new Rotation2d(0));

        // Posições Trench RED ALLIANCE
        public static final Pose2d kInitRedTrechLeft = new Pose2d(13.351, 0.700, new Rotation2d(180));
        public static final Pose2d kFinalRedTrenchLeft = new Pose2d(10.696, 0.700, new Rotation2d(180));
        public static final Pose2d kInitRedTrenchRight = new Pose2d(13.351, 7.400, new Rotation2d(180));
        public static final Pose2d kFinalRedTrenchRight = new Pose2d(10.696, 7.400, new Rotation2d(180));

        // OUTPOST
        // Outpost Blue
        public static final Pose2d kOutPostBlue = new Pose2d(0.567, 0.644, new Rotation2d(0.000));
        // Outpost Red
        public static final Pose2d kOutPostRed = new Pose2d(15.952, 7.405, new Rotation2d(180.000));

        // HUBS
        // Blue
        public static final Translation2d kHubBlue = new Translation2d(4.620, 4.030);
        // Red
        public static final Translation2d kHubRed = new Translation2d(11.910, 4.030);

        // TESTEEEEEEEEEE
        public static final Pose2d kBlueClimb = new Pose2d(1.450, 4.050, new Rotation2d(Math.toRadians(180)));

        public static PathPlannerPath getAutoTrenchPath(Pose2d robotPose, boolean isLeftTrench,
                boolean isFacingForward) {
            var allianceOpt = DriverStation.getAlliance();
            boolean isRed = allianceOpt.isPresent() && allianceOpt.get() == Alliance.Red;

            double blueZoneBoundary = 5.204;
            double redZoneBoundary = 11.285;

            boolean isInBase = isRed ? (robotPose.getX() > redZoneBoundary) : (robotPose.getX() < blueZoneBoundary);

            String pathName = "";
            if (isInBase) {
                if (isLeftTrench) {
                    pathName = isFacingForward ? "BlueLeftTrenchInF" : "BlueLeftTrenchInB";
                } else {
                    pathName = isFacingForward ? "BlueRightTrenchInF" : "BlueRightTrenchInB";
                }
            } else {
                if (isLeftTrench) {
                    pathName = isFacingForward ? "BlueLeftTrenchOutF" : "BlueLeftTrenchOutB";
                } else {
                    pathName = isFacingForward ? "BlueRightTrenchOutF" : "BlueRightTrenchOutB";
                }
            }

            try {
                return PathPlannerPath.fromPathFile(pathName);
            } catch (Exception e) {
                DriverStation.reportError("ERRO: Não foi possível carregar o Path completo: " + pathName,
                        e.getStackTrace());
                return null;
            }
        }
    }
}