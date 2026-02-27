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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Autos.SmartTrench;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOComp;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOComp;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.FieldConstants.FieldPoses;
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
    private final LEDSubsystem led;

    private Pose3d coralPose = new Pose3d(1, 1, 0, new Rotation3d());
    private boolean isHoldingPiece = false;
    private Translation3d coralVelocity = new Translation3d();
    private final double GRAVITY = 9.81;
    private final double SHOOT_SPEED = 12.0;
    private final double SHOOT_PITCH = 45.0;

    private final Joystick joystick1 = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick joystick2 = new Joystick(OIConstants.kSecondDriverControllerPort);

    private final JoystickButton buttonTeste = new JoystickButton(joystick2, 12);

    private final LoggedDashboardChooser<Command> autoChooser;

    // private final PowerDistribution pdh = new PowerDistribution(30,
    // ModuleType.kRev);
    private String currentStateString = "INICIANDO";

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
                        new VisionIOLimelight("limelight-front", drive::getRotation),
                        new VisionIOLimelight("limelight-back", drive::getRotation));

                shooter = new Shooter(new TurretIOComp(), new PivotIOComp(), new FlyWheelIOComp());// , new
                                                                                                   // IntakeIOComp());
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
                shooter = new Shooter(new TurretIOSim(), new PivotIOSim(), new FlyWheelIOSim());// , new IntakeIOSim());
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
                });/*
                    * , new IntakeIO(){
                    * });
                    */
                intake = new Intake(new IntakeIO() {
                });
                break;
        }

        led = new LEDSubsystem(
                () -> shooter.isAutoAimEnabled(),
                () -> !vision.getVisionMeasurements().isEmpty(),
                () -> Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond,
                        drive.getChassisSpeeds().vyMetersPerSecond));

        shooter.setupAutoAimReferences(
                drive::getPose,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
                            ? FieldPoses.kHubRed
                            : FieldPoses.kHubBlue;
                });

        // Intake
        NamedCommands.registerCommand("Intake Deploy", Commands.runOnce(() -> intake.deploy(), intake));
        NamedCommands.registerCommand("Intake Retract", Commands.runOnce(() -> {
            intake.retract();
            intake.stop();
        }, intake));

        // Shooter
        NamedCommands.registerCommand("Shoot", shooter.shootCommand(intake));
        NamedCommands.registerCommand("Spit", shooter.spitCommand(intake));
        NamedCommands.registerCommand("Stop Shooter", Commands.runOnce(() -> shooter.stop(), shooter));
        NamedCommands.registerCommand("Auto Aim", Commands.runOnce(() -> shooter.toggleAutoAim(), shooter));

        NamedCommands.registerCommand("Change Shooter State", Commands.runOnce(() -> changeShootState()));

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> {
                            var alliance = DriverStation.getAlliance();
                            boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                            return isRed ? joystick1.getY() : -joystick1.getY();
                        },
                        () -> {
                            var alliance = DriverStation.getAlliance();
                            boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                            return isRed ? joystick1.getX() : -joystick1.getX();
                        },
                        () -> -joystick1.getRawAxis(4))
                        .alongWith(Commands.run(() -> updateVisionCorrection())));

        // intake.setDefaultCommand(intake.getMaintainPositionCommand());

        Commands.run(this::updateTelemetryAndState)
                .ignoringDisable(true)
                .withName("Telemetria e Estado")
                .schedule();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Driver
        new JoystickButton(joystick1, OIConstants.kIntakeIdx)
                .onTrue(intake.getToggleIntakeCommand());

        new JoystickButton(joystick1, OIConstants.kIntakeFWIdx)
                .onTrue(intake.getToggleRollersCommand());

        new JoystickButton(joystick1, OIConstants.kThroughtTrenchIdx)
                .whileTrue(SmartTrench.run(drive));

        new JoystickButton(joystick1, OIConstants.kResetFrontIdx)
                .onTrue(Commands.runOnce(() -> drive.zeroHeading(), drive));

        // Operador
        new Trigger(() -> joystick2.getRawAxis(3) > 0.5)
                .whileTrue(shooter.shootCommand(intake));

        new JoystickButton(joystick2, OIConstants.kAutoAimIdx)
                .onTrue(Commands.runOnce(() -> shooter.toggleAutoAim(), shooter));

        new JoystickButton(joystick2, OIConstants.kResetTurretEncoderIdx)
                .onTrue(Commands.runOnce(() -> shooter.resetTurretEncoder(), shooter));

        new JoystickButton(joystick2, OIConstants.kResetPivotIdx)
                .onTrue(Commands.runOnce(() -> shooter.resetPivotEncoder(), shooter));

        new POVButton(joystick2, OIConstants.kTurretToLeftPOV)
                .whileTrue(shooter.manualTurretCommand(false));

        new POVButton(joystick2, OIConstants.kTurretToRightPOV)
                .whileTrue(shooter.manualTurretCommand(true));

        new POVButton(joystick2, OIConstants.kPivotUpPOV)
                .whileTrue(shooter.manualPivotCommand(true));

        new POVButton(joystick2, OIConstants.kPivotDownPOV)
                .whileTrue(shooter.manualPivotCommand(false));
        
        new JoystickButton(joystick2, OIConstants.kreverseSystem)
                .whileTrue(new InstantCommand(() -> shooter.reverseSystem()));

        new JoystickButton(joystick2, OIConstants.kreverseSystem)
                .onFalse(new InstantCommand(() -> shooter.reverseOffSystem()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void updateVisionCorrection() {
        var measurements = vision.getVisionMeasurements();
        for (var est : measurements) {
            drive.addVisionMeasurement(est.pose(), est.timestamp(), est.stdDevs());
        }
    }

    public void updateTelemetryAndState() {
        SmartDashboard.putNumber("Energia/TensaoBateria_V", RobotController.getBatteryVoltage());
        // SmartDashboard.putNumber("Energia/CorrenteTotalPDH_A",
        // pdh.getTotalCurrent());
        SmartDashboard.putNumber("RobotState/TempoDePartida", DriverStation.getMatchTime());

        var speeds = drive.getChassisSpeeds();

        double linearSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Velocidade_Mps", linearSpeedMps);

        double angularSpeedDegps = Math.toDegrees(speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Drive/CompensacaoAngular_DegPs", angularSpeedDegps);

        if (joystick1.getRawAxis(3) > 0.5) {
            currentStateString = "ATIRANDO";
        } else if (shooter.isAutoAimEnabled()) {
            currentStateString = "AUTO AIM";
        } else if (joystick1.getRawButton(OIConstants.kIntakeIdx)) {
            currentStateString = "INTAKE";
        } else if (joystick1.getRawButton(OIConstants.kThroughtTrenchIdx)) {
            currentStateString = "ATRAVESSANDO TRENCH";
        } else {
            Pose2d pose = drive.getPose();
            double x = pose.getX();
            double y = pose.getY();

            if (x > 5.8 && x < 10.7) {
                currentStateString = "ZONA NEUTRA";
            } else if (x <= 5.8 && y > 6.0) {
                currentStateString = "LEFT TRENCH";
            } else if (x >= 10.7) {
                currentStateString = "ZONA INIMIGA";
            } else {
                currentStateString = "BASE";
            }
        }
        SmartDashboard.putString("RobotState/AcaoAtual", currentStateString);
    }

    public void simulationPeriodic() {
        if (buttonTeste.getAsBoolean()) {
            isHoldingPiece = true;
            coralVelocity = new Translation3d();
        } else if (isHoldingPiece) {
            isHoldingPiece = false;
            Pose2d robotPose = drive.getPose();
            double turretAngleDeg = shooter.getTurretPosition();
            Rotation2d totalShotAngle = robotPose.getRotation().plus(Rotation2d.fromDegrees(turretAngleDeg));
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
                coralPose = new Pose3d(coralPose.getTranslation().plus(coralVelocity.times(dt)),
                        coralPose.getRotation());
            } else {
                coralVelocity = new Translation3d();
            }
        }
        Logger.recordOutput("Sim/GamePieces/Coral", coralPose);
    }

    public void shootAutoPeriodic(){
        
        if(Constants.ShootAutoEnable == true){
            shooter.shootCommandAuto(intake,shooter.calculatedAutoAimRpm,shooter.kFeederShootRpm, shooter.kCentrifugeShootRpm);
            
        } else {
            shooter.stop();
            
        }
    }

    public void changeShootState(){
        Constants.ShootAutoEnable = !Constants.ShootAutoEnable;
        System.out.print(Constants.ShootAutoEnable);
    }

    public void doWhenAutoInit(){
        shooter.toggleAutoAim();
    }
    
}