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
        // Load the path you want to follow using its name in the GUI
        public static PathPlannerPath kTrenchBlueLeftPathIn;
        public static PathPlannerPath kTrenchBlueRightPathIn;
        public static PathPlannerPath kTrenchRedLeftPathIn;
        public static PathPlannerPath kTrenchRedRightPathIn;

        public static PathPlannerPath kTrenchBlueLeftPathOut;
        public static PathPlannerPath kTrenchBlueRightPathOut;
        public static PathPlannerPath kTrenchRedLeftPathOut;
        public static PathPlannerPath kTrenchRedRightPathOut;

        public static final Optional<Alliance> alliance = DriverStation.getAlliance();

        static {
            try {
                // In
                kTrenchBlueLeftPathIn = PathPlannerPath.fromPathFile("BlueLeftTrenchIn");
                kTrenchBlueRightPathIn = PathPlannerPath.fromPathFile("BlueRightTrenchIn");
                kTrenchRedLeftPathIn = PathPlannerPath.fromPathFile("RedLeftTrenchIn");
                kTrenchRedRightPathIn = PathPlannerPath.fromPathFile("RedRightTrenchIn");

                // Out
                kTrenchBlueLeftPathOut = PathPlannerPath.fromPathFile("BlueLeftTrenchOut");
                kTrenchBlueRightPathOut = PathPlannerPath.fromPathFile("BlueRightTrenchOut");
                kTrenchRedLeftPathOut = PathPlannerPath.fromPathFile("RedLeftTrenchOut");
                kTrenchRedRightPathOut = PathPlannerPath.fromPathFile("RedRightTrenchOut");

            } catch (Exception e) {
                DriverStation.reportError("ERRO: Não foi possível carregar os Paths em FieldPoses!", e.getStackTrace());
            }
        }

        // public static final Pose2d kCoralRight = alliance.get() == Alliance.Blue ?
        // kTrenchBlueRightPath : kTrenchRedLeftPath;
        // public static final Pose2d kCoralLeft = alliance.get() == Alliance.Blue ?
        // kTrenchBlueRightPath : kTrenchRedRightPath;

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

        //HUBS
        //Blue
        public static final Translation2d kHubBlue = new Translation2d(4.620, 4.030);
        //Red
        public static final Translation2d kHubRed = new Translation2d(11.910, 4.030);

        // TESTEEEEEEEEEE
        public static final Pose2d kteste = new Pose2d(5.227, 5.374, new Rotation2d(Math.toRadians(240)));


        //Filtro de saida e entrada das alianças
        public static PathPlannerPath getAutoTrenchPath(Pose2d robotPose, boolean isLeftTrench) {
            var allianceOpt = DriverStation.getAlliance();
            Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;

            double blueZoneBoundary = 5.204;
            double redZoneBoundary = 11.285;

            if (alliance == Alliance.Blue) {
                // aliança azul
                boolean isInBase = robotPose.getX() < blueZoneBoundary;

                if (isInBase) {
                    return isLeftTrench ? kTrenchBlueLeftPathIn : kTrenchBlueRightPathIn;
                } else {
                    return isLeftTrench ? kTrenchBlueLeftPathOut : kTrenchBlueRightPathOut;
                }

            } else {
                // aliança vermelha
                boolean isInBase = robotPose.getX() > redZoneBoundary;

                if (isInBase) {
                    return isLeftTrench ? kTrenchRedLeftPathIn : kTrenchRedRightPathIn;
                } else {
                    return isLeftTrench ? kTrenchRedLeftPathOut : kTrenchRedRightPathOut;
                }
            }
        }
    }
}
