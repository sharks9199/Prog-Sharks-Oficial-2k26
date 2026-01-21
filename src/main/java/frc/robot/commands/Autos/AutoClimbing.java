package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.subsystems.drive.Drive;

// Assumindo que você tem constantes de campo, senão defina manualmente
// import frc.robot.Constants.FieldPoses; 

public class AutoClimbing extends SequentialCommandGroup {

    /**
     * @param swerve 
     * @param targetPose
     */
    public AutoClimbing(Drive swerve, Pose2d targetPose) {
        
        Pose2d approachPose = targetPose.transformBy(
            new Transform2d(new Translation2d(-1.0, 0), new Rotation2d(0))
        );


        PathConstraints fastConstraints = new PathConstraints(
            4.0, 3.0,
            Math.PI * 4, Math.PI * 8 
        );

        PathConstraints dockingConstraints = new PathConstraints(
            1.0, 1.0,
            Math.PI, Math.PI * 2
        );

        addCommands(
            AutoBuilder.pathfindToPose(approachPose, fastConstraints),
            AutoBuilder.pathfindToPose(targetPose, dockingConstraints)
        );
    }
}