package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d; // <--- Certifique-se de importar isso
// Importe seu DriveSubsystem aqui se necessário
// import frc.robot.subsystems.drive.Drive; 

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    // Referência para o Drive (opcional, depende de como você estrutura)
    // private final Drive drive;

    public Vision(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // 1. Ler dados do IO
        io.updateInputs(inputs);

        // 2. Registrar no AdvantageScope
        Logger.processInputs("Vision", inputs);

        // Registrar poses para visualização 3D
        if (inputs.hasTarget) {
            // inputs.robotPose já é Pose3d (definido no VisionIO)
            Logger.recordOutput("Vision/RobotPose", inputs.robotPose);
        } else {
            // --- CORREÇÃO AQUI ---
            // Antes estava new Pose2d(). Mudamos para Pose3d() para manter o tipo
            // consistente.
            Logger.recordOutput("Vision/RobotPose", new Pose3d());
        }
    }

    /**
     * Retorna a estimativa de pose para ser usada no SwerveDrivePoseEstimator.
     * Chame este método dentro do periodic do seu DriveSubsystem.
     */
/**
     * Retorna a estimativa de pose para ser usada no SwerveDrivePoseEstimator.
     */
    public java.util.Optional<VisionMeasurement> getVisionMeasurement() {
        if (!inputs.hasTarget || inputs.tagCount == 0) {
            return java.util.Optional.empty();
        }

        double xyStdDev;
        double thetaStdDev; // Confiança na Rotação
        
        // --- LÓGICA DE CONFIANÇA ---
        
        if (inputs.tagCount >= 2) {
            // MÚLTIPLAS TAGS: Confiança EXTREMA
            // A triangulação é muito precisa para X, Y e Rotação.
            xyStdDev = 0.02; 
            thetaStdDev = 0.05; 
        } 
        else if (inputs.avgTagDist > 4.0) {
            // TAG ÚNICA E LONGE (> 4 metros)
            // Aqui a leitura é muito ruidosa, mas se você quer forçar a atualização,
            // usaremos valores mais altos (menos confiança que de perto).
            xyStdDev = 1.0; 
            thetaStdDev = 3.0; // Confia pouco, mas corrige se o erro for grande
        } 
        else {
            // TAG ÚNICA E PERTO (< 4 metros)
            // É aqui que você queria a mudança.
            xyStdDev = 0.1;
            
            // --- MUDANÇA AQUI ---
            // Antes estava 999999. Agora colocamos um valor "médio".
            // Isso permite que a visão corrija o giroscópio devagar.
            thetaStdDev = 1.5; 
        }

        // Cria a matriz: [X, Y, Rotação]
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

        return java.util.Optional.of(new VisionMeasurement(
            inputs.robotPose.toPose2d(),
            inputs.timestamp,
            stdDevs
        ));
    }

    // --- Métodos Auxiliares (Getters para seus comandos antigos) ---

    public double getTX() {
        return inputs.tx;
    }

    public double getTY() {
        return inputs.ty;
    }

    public boolean hasTarget() {
        return inputs.hasTarget;
    }

    public void setPipeline(int pipeline) {
        io.setPipeline(pipeline);
    }

    // Record auxiliar para passar dados para o Drive
    public record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    }
}