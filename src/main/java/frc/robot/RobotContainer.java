package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Autos.AutoClimbing;
import frc.robot.generated.TunerConstants;
// SUBSYSTEMS DRIVE
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOComp;
import frc.robot.subsystems.drive.ModuleIOSim;
// IMPORTANTE: Imports da Visão
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.FieldConstants.FieldPoses;

public class RobotContainer {

  private final Drive drive;
  // private final Shooter shooter;

  // 1. Declarar a Visão
  private final Vision vision;

  // SIMULAÇÃO
  private Pose3d coralPose = new Pose3d(1, 1, 0, new Rotation3d());
  private boolean isHoldingPiece = false;
  private Translation3d coralVelocity = new Translation3d();
  private final double GRAVITY = 9.81;
  private final double SHOOT_SPEED = 12.0;
  private final double SHOOT_PITCH = 45.0;

  // CONTROLLER
  private final Joystick joystick = new Joystick(0);
  private final JoystickButton buttonA = new JoystickButton(joystick, 2);
  private final JoystickButton buttonB = new JoystickButton(joystick, 3);
  private final JoystickButton buttonX = new JoystickButton(joystick, 1);

  private final POVButton povUp = new POVButton(joystick, 0);
  private final POVButton povDown = new POVButton(joystick, 180);
  private final POVButton povLeft = new POVButton(joystick, 270);
  private final POVButton povRight = new POVButton(joystick, 90);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOComp(TunerConstants.FrontLeft),
            new ModuleIOComp(TunerConstants.FrontRight),
            new ModuleIOComp(TunerConstants.BackLeft),
            new ModuleIOComp(TunerConstants.BackRight));

        // shooter = new Shooter(
        // new FlyWheelIOComp(ShooterConstants.FlyWheelConstants.kFWMotor),
        // new PivotIOComp(ShooterConstants.PivotConstants.kPivotMotor),
        // new TurretIOComp(ShooterConstants.TurretConstants.kTurretMotor));

        // --- CORREÇÃO AQUI ---
        // A inicialização deve vir ANTES do break
        vision = new Vision(new VisionIOLimelight("limelight-reefle", drive::getRotation));
        break;

      case SIM:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

        // shooter = new Shooter(
        // new FlyWheelIOSim(),
        // new PivotIOSim(),
        // new TurretIOSim());

        // --- CORREÇÃO AQUI ---
        // Inicialização antes do break
        vision = new Vision(new VisionIO() {
        });
        break;

      default:
        // Replay
        drive = new Drive(new GyroIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        });

        // shooter = new Shooter(
        // new FlyWheelIO() {
        // },
        // new PivotIO() {
        // },
        // new TurretIO() {
        // });

        // --- CORREÇÃO AQUI ---
        // Você PRECISA inicializar a visão no default também,
        // senão o Java reclama que a variável 'final' pode estar vazia.
        vision = new Vision(new VisionIO() {
        });
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -joystick.getY(),
            () -> -joystick.getX(),
            () -> joystick.getZ())
            .alongWith(Commands.run(() -> updateVisionCorrection())));
    // CONTROLE MANUAL DO SHOOTER
    /*
     * shooter.setDefaultCommand(
     * Commands.run(() -> {
     * double turretV = 0.0;
     * double pivotV = 0.0;
     * double flyV = 0.0;
     * 
     * if (povLeft.getAsBoolean())
     * turretV = -4.0;
     * if (povRight.getAsBoolean())
     * turretV = 4.0;
     * 
     * if (povUp.getAsBoolean())
     * pivotV = 3.0;
     * if (povDown.getAsBoolean())
     * pivotV = -3.0;
     * 
     * if (buttonA.getAsBoolean())
     * flyV = 12.0;
     * 
     * shooter.setInputs(turretV, pivotV, flyV);
     * }, shooter));
     */

    // Botão para resetar o Giroscópio (Definir "Frente")
    buttonX.onTrue(Commands.runOnce(
        () -> drive.zeroHeading(),
        drive).ignoringDisable(true));

    // Botão B (Joystick 3)
    new JoystickButton(joystick, 3)
        .onTrue(drive.pathfindToPose(
            () -> FieldPoses.kteste));

    new JoystickButton(joystick, 2)
        .whileTrue(
            new AutoClimbing(
                drive,
                new Pose2d(5.227, 5.374, new Rotation2d(Math.toRadians(240)))
            ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateVisionCorrection() {
    // 1. Pega a medição do subsistema Vision
    var visionEst = vision.getVisionMeasurement();

    // 2. Se houver uma medição válida, envia para o Drive
    if (visionEst.isPresent()) {
      var est = visionEst.get();

      // Passa a pose, o timestamp e o desvio padrão (confiança)
      drive.addVisionMeasurement(
          est.pose(),
          est.timestamp(),
          est.stdDevs());
    }
  }

  public void simulationPeriodic() {

    /* shooter.updateVisualizer(drive.getPose()); */

    if (buttonA.getAsBoolean()) {
      isHoldingPiece = true;
      coralVelocity = new Translation3d();
    } else if (isHoldingPiece) {
      isHoldingPiece = false;
      Pose2d robotPose = drive.getPose();

      double pitchRad = Math.toRadians(SHOOT_PITCH);
      double vXY = SHOOT_SPEED * Math.cos(pitchRad);

      coralVelocity = new Translation3d(
          vXY * Math.cos(robotPose.getRotation().getRadians()),
          vXY * Math.sin(robotPose.getRotation().getRadians()),
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