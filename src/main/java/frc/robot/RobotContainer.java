package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TurretManualCmd;
import frc.robot.commands.Autos.AutoAim;
import frc.robot.commands.Autos.SmartTrench;
import frc.robot.generated.TunerConstants;

// SUBSYSTEMS
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOComp;
import frc.robot.subsystems.Intake.IntakeIOSim;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOComp;
import frc.robot.subsystems.drive.ModuleIOSim;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIO;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIOComp;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIOSim;
import frc.robot.subsystems.shooter.Pivot.PivotIO;
import frc.robot.subsystems.shooter.Pivot.PivotIOComp;
import frc.robot.subsystems.shooter.Pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Turret.TurretIO;
import frc.robot.subsystems.shooter.Turret.TurretIOComp;
import frc.robot.subsystems.shooter.Turret.TurretIOSim;

public class RobotContainer {

    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Intake intake;

    // SIMULAÇÃO
    private Pose3d coralPose = new Pose3d(1, 1, 0, new Rotation3d());
    private boolean isHoldingPiece = false;
    private Translation3d coralVelocity = new Translation3d();
    private final double GRAVITY = 9.81;
    private final double SHOOT_SPEED = 12.0;
    private final double SHOOT_PITCH = 45.0;

    // CONTROLLER
    private final Joystick joystick1 = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick joystick2 = new Joystick(OIConstants.kSecondDriverControllerPort);

    // Botão extra de teste no controle 2
    private final JoystickButton buttonTeste = new JoystickButton(joystick2, 12);

    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {

        // --- INICIALIZAÇÃO DOS SUBSYSTEMS ---
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOComp(TunerConstants.FrontLeft),
                        new ModuleIOComp(TunerConstants.FrontRight),
                        new ModuleIOComp(TunerConstants.BackLeft),
                        new ModuleIOComp(TunerConstants.BackRight));

                // --- ATUALIZADO: DUAS LIMELIGHTS ---
                vision = new Vision(
                        new VisionIOLimelight("limelight-front", drive::getRotation),
                        new VisionIOLimelight("limelight-back", drive::getRotation)
                );

                shooter = new Shooter(new TurretIOComp(), new PivotIOComp(), new FlyWheelIOComp());
                intake = new Intake(new IntakeIOComp());
                break;

            case SIM:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                vision = new Vision(new VisionIO() {});
                shooter = new Shooter(new TurretIOSim(), new PivotIOSim(), new FlyWheelIOSim());
                intake = new Intake(new IntakeIOSim());
                break;

            default:
                drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = new Vision(new VisionIO() {});
                shooter = new Shooter(new TurretIO() {}, new PivotIO() {}, new FlyWheelIO() {});
                intake = new Intake(new IntakeIO() {});
                break;
        }

        // =================================================================
        // NAMED COMMANDS (Para usar no PathPlanner)
        // =================================================================

        NamedCommands.registerCommand("Intake Start", Commands.runOnce(() -> intake.getToggleIntakeCommand().schedule()));
        NamedCommands.registerCommand("Intake Stop", Commands.runOnce(() -> intake.stop(), intake));

        NamedCommands.registerCommand("Shoot", shooter.shootCommand());
        NamedCommands.registerCommand("Spit", shooter.spitCommand());
        NamedCommands.registerCommand("Stop Shooter", Commands.runOnce(() -> shooter.stop(), shooter));

        NamedCommands.registerCommand("SpinUp Flywheel", Commands.runOnce(() -> shooter.setFlywheelVelocity(3500.0), shooter));

        NamedCommands.registerCommand("Auto Aim", new AutoAim(shooter, drive::getPose));
        NamedCommands.registerCommand("Reset Turret", Commands.runOnce(() -> shooter.setTurretSetpoint(0.0), shooter));

        // Build Auto Chooser
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // =================================================================
        // DEFAULT COMMANDS (Comandos que rodam sempre que nada mais está rodando)
        // =================================================================

        // 1. DRIVE
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -joystick1.getY(),
                () -> -joystick1.getX(),
                () -> joystick1.getZ())
                .alongWith(Commands.run(() -> updateVisionCorrection())));

        // 2. SHOOTER (Torre Manual)
        shooter.setDefaultCommand(
            new TurretManualCmd(shooter, () -> joystick1.getPOV()));

        // 3. INTAKE
        intake.setDefaultCommand(intake.getMaintainPositionCommand());

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // =================================================================
        // BOTÕES - AÇÕES
        // =================================================================

        // --- TIRO E MANIPULAÇÃO ---
        new JoystickButton(joystick1, OIConstants.kShootidx)
                .whileTrue(shooter.shootCommand());

        new JoystickButton(joystick1, OIConstants.kAmpIdx)
                .whileTrue(shooter.spitCommand());

        new JoystickButton(joystick1, OIConstants.kAutoAimIdx)
                .whileTrue(new AutoAim(shooter, drive::getPose));

        // --- INTAKE TOGGLE ---
        new JoystickButton(joystick1, OIConstants.kIntakeIdx)
                .onTrue(intake.getToggleIntakeCommand());

        new JoystickButton(joystick1, OIConstants.kThroughtTrenchIdx)
                .whileTrue(SmartTrench.run(drive));


        // --- RESET E CONFIGURAÇÃO ---
        new JoystickButton(joystick1, OIConstants.kResetFrontIdx)
                .onTrue(Commands.runOnce(() -> drive.zeroHeading(), drive));

        new JoystickButton(joystick2, OIConstants.kResetTurretEncoderIdx)
                .onTrue(Commands.runOnce(() -> shooter.resetTurretEncoder(), shooter));

        new JoystickButton(joystick2, OIConstants.kResetPivotIdx)
                .onTrue(Commands.runOnce(() -> shooter.resetPivotEncoder(), shooter));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Atualiza a odometria com dados de visão (Múltiplas câmeras).
     */
    public void updateVisionCorrection() {
        // Pega a lista de todas as medições de todas as câmeras
        var measurements = vision.getVisionMeasurements();

        // Itera sobre cada medição válida e adiciona ao estimador de pose
        for (var est : measurements) {
            drive.addVisionMeasurement(
                    est.pose(),
                    est.timestamp(),
                    est.stdDevs());
        }
    }

    public void simulationPeriodic() {
        if (buttonTeste.getAsBoolean()) {
            isHoldingPiece = true;
            coralVelocity = new Translation3d();
        } else if (isHoldingPiece) {
            isHoldingPiece = false;
            Pose2d robotPose = drive.getPose();
            double turretAngleDeg = shooter.getTurretPosition();
            Rotation2d totalShotAngle = robotPose.getRotation()
                    .plus(Rotation2d.fromDegrees(turretAngleDeg));
            double pitchRad = Math.toRadians(SHOOT_PITCH);
            double vXY = SHOOT_SPEED * Math.cos(pitchRad);

            coralVelocity = new Translation3d(
                    vXY * totalShotAngle.getCos(),
                    vXY * totalShotAngle.getSin(),
                    SHOOT_SPEED * Math.sin(pitchRad));
        }

        if (isHoldingPiece) {
            Pose2d p = drive.getPose();
            coralPose = new Pose3d(p.getX(), p.getY(), 0.5, new Rotation3d(0, 0, p.getRotation().getRadians()))
                    .transformBy(new Transform3d(new Translation3d(0.2, 0, 0), new Rotation3d()));
        } else {
            if (coralPose.getZ() > 0.0) {
                double dt = 0.02;
                coralVelocity = coralVelocity.minus(new Translation3d(0, 0, GRAVITY * dt));
                coralPose = new Pose3d(coralPose.getTranslation().plus(coralVelocity.times(dt)), coralPose.getRotation());
            } else {
                coralVelocity = new Translation3d();
            }
        }
        Logger.recordOutput("Sim/GamePieces/Coral", coralPose);
    }
}