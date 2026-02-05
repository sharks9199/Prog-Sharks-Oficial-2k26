package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {
    private final String name;
    private final Supplier<Rotation2d> rotationSupplier;

    public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
        this.name = name;
        this.rotationSupplier = rotationSupplier;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(name);
        inputs.ty = LimelightHelpers.getTY(name);
        inputs.ta = LimelightHelpers.getTA(name);
        inputs.fiducialID = (long) LimelightHelpers.getFiducialID(name);

        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (mt2 != null) {
            inputs.hasTarget = mt2.tagCount > 0;

            inputs.robotPose = new Pose3d(mt2.pose);

            inputs.timestamp = mt2.timestampSeconds;
            inputs.tagCount = mt2.tagCount;
            inputs.avgTagDist = mt2.avgTagDist;
        } else {
            inputs.hasTarget = false;
            inputs.robotPose = new Pose3d();
            inputs.tagCount = 0;
        }
    }

    @Override
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(name, pipeline);
    }
}