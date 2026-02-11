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
import java.util.Optional;

public class Vision extends SubsystemBase {
    // Agora aceitamos múltiplas câmeras (Array)
    private final VisionIO[] ios;
    private final VisionIOInputsAutoLogged[] inputs;

    // Construtor aceita VARARGS (VisionIO... ios)
    // Isso permite passar: new Vision(cam1) OU new Vision(cam1, cam2, cam3...)
    public Vision(VisionIO... ios) {
        this.ios = ios;
        this.inputs = new VisionIOInputsAutoLogged[ios.length];
        
        // Inicializa os inputs para cada câmera
        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        // Loop para atualizar cada câmera individualmente
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            
            // Loga separado: "Vision/Inst0", "Vision/Inst1", etc.
            Logger.processInputs("Vision/Inst" + i, inputs[i]);

            if (inputs[i].hasTarget) {
                Logger.recordOutput("Vision/Inst" + i + "/RobotPose", inputs[i].robotPose);
            } else {
                Logger.recordOutput("Vision/Inst" + i + "/RobotPose", new Pose3d());
            }
        }
    }

    /**
     * Retorna uma LISTA de medições.
     * O DriveSubsystem deve iterar sobre essa lista e adicionar cada uma.
     */
    public List<VisionMeasurement> getVisionMeasurements() {
        List<VisionMeasurement> measurements = new ArrayList<>();

        for (VisionIOInputsAutoLogged input : inputs) {
            // Se essa câmera não tem alvo ou pose válida, pula
            if (!input.hasTarget || input.tagCount == 0) {
                continue;
            }

            double xyStdDev;
            double thetaStdDev;

            // Lógica de desvio padrão (Confiança)
            if (input.tagCount >= 2) {
                xyStdDev = 0.02;
                thetaStdDev = 0.05;
            } else if (input.avgTagDist > 4.0) {
                xyStdDev = 1.0;
                thetaStdDev = 3.0;
            } else {
                xyStdDev = 0.1;
                thetaStdDev = 1.5;
            }

            // CORREÇÃO: Usando as variáveis calculadas acima
            Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

            measurements.add(new VisionMeasurement(
                input.robotPose.toPose2d(),
                input.timestamp,
                stdDevs
            ));
        }

        return measurements;
    }

    public double getBestTX() {
        for (VisionIOInputsAutoLogged input : inputs) {
            if (input.hasTarget) return input.tx;
        }
        return 0.0;
    }

    public double getBestTY() {
        for (VisionIOInputsAutoLogged input : inputs) {
            if (input.hasTarget) return input.ty;
        }
        return 0.0;
    }

    public boolean hasTarget() {
        for (VisionIOInputsAutoLogged input : inputs) {
            if (input.hasTarget) return true;
        }
        return false;
    }

    public record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}
}