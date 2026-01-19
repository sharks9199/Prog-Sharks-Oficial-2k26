package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d; // <--- NOVO
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIO.FlyWheelIOInputs;
import frc.robot.subsystems.shooter.Pivot.PivotIO.PivotIOInputs;
import frc.robot.subsystems.shooter.Turret.TurretIO.TurretIOInputs;

public class ShooterVisualizer {

    private final Mechanism2d mechanism;
    private final MechanismRoot2d root;
    private final MechanismLigament2d turretLigament;
    private final MechanismLigament2d pivotLigament;
    private final MechanismLigament2d flywheelLigament;

    // Ajuste aqui a posição da Torre NO ROBÔ (X=Frente, Z=Altura)
    private final Transform3d robotToTurret = new Transform3d(
        new Translation3d(0.0, 0.0, 0.2), // Ex: 20cm de altura
        new Rotation3d(0, 0, 0)
    );

    private final Transform3d turretToPivot = new Transform3d(
        new Translation3d(0.0, 0.0, 0.3), 
        new Rotation3d(0, 0, 0)
    );

    private final PivotIOInputs pivotInputs;
    private final FlyWheelIOInputs flywheelInputs;
    private final TurretIOInputs turretInputs;

    public ShooterVisualizer(PivotIOInputs pivotInputs, FlyWheelIOInputs flywheelInputs, TurretIOInputs turretInputs) {
        this.pivotInputs = pivotInputs;
        this.flywheelInputs = flywheelInputs;
        this.turretInputs = turretInputs;

        this.mechanism = new Mechanism2d(3.0, 3.0);
        this.root = mechanism.getRoot("ShooterBase", 1.5, 0.5);
        
        this.turretLigament = root.append(
            new MechanismLigament2d("Turret", 0.5, 90, 6, new Color8Bit(Color.kBlue))
        );
        this.pivotLigament = turretLigament.append(
            new MechanismLigament2d("Pivot", 1.0, 45, 4, new Color8Bit(Color.kYellow))
        );
        this.flywheelLigament = pivotLigament.append(
            new MechanismLigament2d("Flywheel", 0.2, 0, 10, new Color8Bit(Color.kRed))
        );
        
        SmartDashboard.putData("Shooter/Mechanism2d", mechanism);
    }

    // --- MUDANÇA AQUI: Agora recebe a Pose do Robô ---
    public void update(Pose2d robotPose) {
        
        // 1. Atualiza 2D
        turretLigament.setAngle(Units.radiansToDegrees(turretInputs.positionRads) + 90);
        pivotLigament.setAngle(Units.radiansToDegrees(pivotInputs.positionRads));
        
        double speed = Math.abs(flywheelInputs.velocityRadsPerSec);
        if (speed > 50.0) flywheelLigament.setColor(new Color8Bit(Color.kLimeGreen));
        else if (speed > 10.0) flywheelLigament.setColor(new Color8Bit(Color.kOrange));
        else flywheelLigament.setColor(new Color8Bit(Color.kRed));
        
        SmartDashboard.putData("Shooter/Mechanism2d", mechanism);

        // 2. Atualiza 3D (Calculando a partir da posição do robô)
        
        // Cria uma Pose3d baseada onde o robô está no campo
        Pose3d robotPose3d = new Pose3d(robotPose);

        // Calcula onde está a Torre (Robô + Transform + Rotação da Torre)
        Pose3d turretPose = robotPose3d
            .transformBy(robotToTurret)
            .transformBy(new Transform3d(
                new Translation3d(), 
                new Rotation3d(0, 0, turretInputs.positionRads)
            ));

        // Calcula onde está o Pivot (Torre + Transform + Inclinação)
        Pose3d pivotPose = turretPose
            .transformBy(turretToPivot)
            .transformBy(new Transform3d(
                new Translation3d(),
                new Rotation3d(0, -pivotInputs.positionRads, 0)
            ));

        Logger.recordOutput("Shooter/3D/TurretPose", turretPose);
        Logger.recordOutput("Shooter/3D/PivotPose", pivotPose);
    }
}