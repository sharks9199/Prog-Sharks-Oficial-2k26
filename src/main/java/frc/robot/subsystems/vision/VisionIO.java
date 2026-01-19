package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTarget = false;
        public Pose3d robotPose = new Pose3d();
        public double timestamp = 0.0;
        public int tagCount = 0;
        public double avgTagDist = 0.0;
        public long fiducialID = -1;
        
        public double tx = 0.0;
        public double ty = 0.0;
        public double ta = 0.0;

        public double pipelineIndex = 0.0;
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default void setPipeline(int pipeline) {}
}