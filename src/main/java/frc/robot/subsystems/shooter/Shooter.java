package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;

// --- MUDANÇA NOS IMPORTS ---
// Agora importamos a versão "AutoLogged" que foi gerada pelo Build
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIO;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIOInputsAutoLogged; // <--- NOVO

import frc.robot.subsystems.shooter.Pivot.PivotIO;
import frc.robot.subsystems.shooter.Pivot.PivotIOInputsAutoLogged; // <--- NOVO

import frc.robot.subsystems.shooter.Turret.TurretIO;
import frc.robot.subsystems.shooter.Turret.TurretIOInputsAutoLogged; // <--- NOVO

public class Shooter extends SubsystemBase {

    // --- HARDWARE ---
    private final FlyWheelIO flyWheelIO;
    private final PivotIO pivotIO;
    private final TurretIO turretIO;

    // --- INPUTS (Use a versão AutoLogged!) ---
    // O "AutoLogged" é a classe real que o Logger aceita.
    private final FlyWheelIOInputsAutoLogged flyWheelInputs = new FlyWheelIOInputsAutoLogged();
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

    // Visualizer
    private final ShooterVisualizer visualizer;

    // Estado
    private boolean flywheelModoAutomatico = false;

    // CONSTRUTOR
    public Shooter(FlyWheelIO flyWheelIO, PivotIO pivotIO, TurretIO turretIO) {
        this.flyWheelIO = flyWheelIO;
        this.pivotIO = pivotIO;
        this.turretIO = turretIO;

        // Visualizer (ele aceita os inputs normais ou autologged, pois um herda do
        // outro)
        this.visualizer = new ShooterVisualizer(pivotInputs, flyWheelInputs, turretInputs);
    }

    @Override
    public void periodic() {
        // 1. ATUALIZA TODAS AS LEITURAS
        flyWheelIO.updateInputs(flyWheelInputs);
        pivotIO.updateInputs(pivotInputs);
        turretIO.updateInputs(turretInputs);

        // 2. MANDA PRO LOG (Agora funciona!)
        // O Logger aceita porque "AutoLogged" implementa LoggableInputs
        Logger.processInputs("Shooter/FlyWheel", flyWheelInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Turret", turretInputs);
    }

// Método novo para atualizar visualização
    public void updateVisualizer(Pose2d robotPose) {
        visualizer.update(robotPose);
    }

    // ... O resto do código (setInputs, toggleFlywheel, etc) continua IGUAL ...
    public void setInputs(double turretVolts, double pivotVolts, double gatilhoVolts) {
        turretIO.runVolts(Volts.of(turretVolts));
        pivotIO.runVolts(Volts.of(pivotVolts));

        if (flywheelModoAutomatico) {
            flyWheelIO.runVolts(Volts.of(12.0));
        } else {
            flyWheelIO.runVolts(Volts.of(gatilhoVolts));
        }
    }

    public void toggleFlywheel() {
        flywheelModoAutomatico = !flywheelModoAutomatico;
    }

    public void stop() {
        flywheelModoAutomatico = false;
        flyWheelIO.stop();
        pivotIO.stop();
        turretIO.stop();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}