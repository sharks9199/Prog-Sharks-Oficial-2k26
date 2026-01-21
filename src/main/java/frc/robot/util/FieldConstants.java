package frc.robot.util;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
        public static final class FieldPoses {
        // Load the path you want to follow using its name in the GUI
        public static PathPlannerPath kCoralBlueLeftPath;
        public static PathPlannerPath kCoralBlueRightPath;

            //TRENCHS
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

            //OUTPOST
        //Outpost Blue
        public static final Pose2d kOutPostBlue = new Pose2d(0.567, 0.644, new Rotation2d(0.000));
        //Outpost Red
        public static final Pose2d kOutPostRed = new Pose2d(15.952, 7.405, new Rotation2d(180.000));
        




        //Ainda n sei
        public static final Optional<Alliance> alliance = DriverStation.getAlliance();
        
        //public static final Pose2d kCoralRight = alliance.get() == Alliance.Blue ? kCoralBlueRight : kCoralRedRight;
        //public static final Pose2d kCoralLeft = alliance.get() == Alliance.Blue ? kCoralBlueLeft : kCoralRedLeft;
        
            //TESTEEEEEEEEEE
            public static final Pose2d kteste = new Pose2d(5.227, 5.374, new Rotation2d(Math.toRadians(240)));
    }
}
