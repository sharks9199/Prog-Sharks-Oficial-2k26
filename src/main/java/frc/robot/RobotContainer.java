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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TurretManualCmd;
import frc.robot.commands.Autos.AutoAim;
import frc.robot.commands.Autos.SmartTrench;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOComp;
import frc.robot.subsystems.Intake.IntakeIOSim;
// SUBSYSTEMS DRIVE
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOComp;
import frc.robot.subsystems.drive.ModuleIOSim;

// SUBSYSTEMS VISION
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

// SUBSYSTEMS SHOOTER
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

  private final JoystickButton buttonTeste = new JoystickButton(joystick2, 12);

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

        vision = new Vision(
            new VisionIOLimelight("limelight-front", drive::getRotation));

        shooter = new Shooter(new TurretIOComp(), new PivotIOComp(), new FlyWheelIOComp());

        intake = new Intake(new IntakeIOComp());

        break;

      case SIM:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

        vision = new Vision(new VisionIO() {
        });

        shooter = new Shooter(new TurretIOSim(), new PivotIOSim(), new FlyWheelIOSim());

        intake = new Intake(new IntakeIOSim());
        break;

      default:
        drive = new Drive(new GyroIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        });
        vision = new Vision(new VisionIO() {
        });
        shooter = new Shooter(new TurretIO() {
        }, new PivotIO() {
        }, new FlyWheelIO() {
        });

        intake = new Intake(new IntakeIO() {
        });
        break;
    }
    NamedCommands.registerCommand("AutoAim", new AutoAim(shooter, drive::getPose));
    NamedCommands.registerCommand(null, getAutonomousCommand());

    // SEU CÓDIGO EXISTENTE:
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // =================================================================
    // DEFAULT COMMANDS
    // =================================================================

    // 1. DRIVE
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -joystick1.getY(),
            () -> -joystick1.getX(),
            () -> joystick1.getZ())
            .alongWith(Commands.run(() -> updateVisionCorrection())));

    shooter.setDefaultCommand(
        new TurretManualCmd(shooter, () -> joystick1.getPOV()));

    new JoystickButton(joystick1, OIConstants.kThroughtTrenchIdx)
        .whileTrue(SmartTrench.run(drive));

    new JoystickButton(joystick1, OIConstants.kResetFrontIdx)
        .onTrue(Commands.runOnce(() -> drive.zeroHeading(), drive));

    new JoystickButton(joystick1, OIConstants.kAutoAimIdx)
        .whileTrue(new AutoAim(shooter, drive::getPose));

    new JoystickButton(joystick1, OIConstants.kResetTurretEncoderIdx)
        .onTrue(Commands.runOnce(() -> shooter.resetTurretEncoder(), shooter));

    new JoystickButton(joystick1, OIConstants.kIntakeIdx)
        .onTrue(Commands.runOnce(() -> shooter.resetPivotEnconder(), shooter));

    new JoystickButton(joystick1, OIConstants.kIntakeIdx)
        .onTrue(intake.getCollectCommand(() -> joystick1.getRawButton(OIConstants.kIntakeIdx)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateVisionCorrection() {
    var visionEst = vision.getVisionMeasurement();

    if (visionEst.isPresent()) {
      var est = visionEst.get();

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