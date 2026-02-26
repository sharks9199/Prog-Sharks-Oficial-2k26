package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
    private final VisionIO[] ios;
    private final VisionIOInputsAutoLogged[] inputs;

    public Vision(VisionIO... ios) {
        this.ios = ios;
        this.inputs = new VisionIOInputsAutoLogged[ios.length];
        
        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            
            Logger.processInputs("Vision/Inst" + i, inputs[i]);

            if (inputs[i].hasTarget) {
                Logger.recordOutput("Vision/Inst" + i + "/RobotPose", inputs[i].robotPose);
            } else {
                Logger.recordOutput("Vision/Inst" + i + "/RobotPose", new Pose3d());
            }
        }
    }

    private VisionIOInputsAutoLogged getBestInput() {
        VisionIOInputsAutoLogged bestInput = null;
        double minDistance = Double.MAX_VALUE;

        for (VisionIOInputsAutoLogged input : inputs) {
            if (input.hasTarget && input.tagCount > 0 && input.avgTagDist < minDistance) {
                bestInput = input;
                minDistance = input.avgTagDist;
            }
        }
        
        return bestInput;
    }
    public List<VisionMeasurement> getVisionMeasurements() {
        List<VisionMeasurement> measurements = new ArrayList<>();
        
        VisionIOInputsAutoLogged bestInput = getBestInput();

        if (bestInput == null) {
            return measurements;
        }

        double xyStdDev;
        double thetaStdDev;

        if (bestInput.tagCount >= 2) {
            xyStdDev = 0.02;
            thetaStdDev = 0.05;
        } else if (bestInput.avgTagDist > 4.0) {
            xyStdDev = 1.0;
            thetaStdDev = 3.0;
        } else {
            xyStdDev = 0.1;
            thetaStdDev = 1.5;
        }

        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

        measurements.add(new VisionMeasurement(
            bestInput.robotPose.toPose2d(),
            bestInput.timestamp,
            stdDevs
        ));

        return measurements;
    }
    public double getBestTX() {
        VisionIOInputsAutoLogged best = getBestInput();
        return best != null ? best.tx : 0.0;
    }

    public double getBestTY() {
        VisionIOInputsAutoLogged best = getBestInput();
        return best != null ? best.ty : 0.0;
    }

    public boolean hasTarget() {
        return getBestInput() != null;
    }

    public record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}
}