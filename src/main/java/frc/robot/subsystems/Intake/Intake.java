package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakeConstants.intakeConstants;
import frc.robot.subsystems.shooter.Shooter;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private boolean isIntakeActive = false;
    private boolean isRollerActive = false;
    private boolean isFlipped = false;
    private boolean wasShooting = false;
    private boolean isRollerReversed = false;

    // NOVO TOGGLE: Controla se o intake faz o movimento de empurrar durante o tiro
    private boolean enableShootingMotion = true;

    private final ProfiledPIDController upController = new ProfiledPIDController(
            1.5, 0.0, 0.00001,
            new TrapezoidProfile.Constraints(700, 600));

    private final ProfiledPIDController downController = new ProfiledPIDController(
            1, 0.0, 0.00001,
            new TrapezoidProfile.Constraints(700, 700));

    public Intake(IntakeIO io) {
        this.io = io;
        intakeConstants.intakeSetpoint = intakeConstants.StowedPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        SmartDashboard.putNumber("Intake/IntakePosition", getPosition());
        SmartDashboard.putNumber("Intake/IntakeSpeed", getSpeed());
        SmartDashboard.putNumber("Intake/Intake Setpoint", getSetpoint());
        SmartDashboard.putBoolean("Intake/Intake Is Active", isIntakeActive);
        SmartDashboard.putBoolean("Intake/Rollers Are Active", isRollerActive);
        SmartDashboard.putBoolean("Intake/Shooting Motion Enabled", enableShootingMotion); // Feedback no Dashboard

        SmartDashboard.putNumber("Intake/StowedPosition", intakeConstants.StowedPosition);
        SmartDashboard.putNumber("Intake/CollectPosition", intakeConstants.CollectPosition);
        SmartDashboard.putNumber("Intake/ABSOLUTE POSITION REAL", getPosition());
        SmartDashboard.putNumber("Intake/IntakeApplied Volts", inputs.rotationAppliedVolts);
        SmartDashboard.putNumber("Intake/IntakeCurrent Amps", inputs.rotationCurrentAmps);

        if (Shooter.isShooting) {
            IntakeShooting();
        } else if (wasShooting) {
            if (isIntakeActive) {
                changeSetpoint(intakeConstants.CollectPosition);
            } else {
                changeSetpoint(intakeConstants.StowedPosition);
            }

            isFlipped = false;
        }
        wasShooting = Shooter.isShooting;

        double currentPos = getPosition();
        double target = getSetpoint();
        double output = 0;

        if (target > currentPos) {
            output = downController.calculate(currentPos, target);
        } else {
            output = upController.calculate(currentPos, target);
        }

        setPlanetary(output);
    }

    // --- GETTERS ---
    public double getPosition() {
        return inputs.rotationPosition;
    }

    public double getSpeed() {
        return inputs.rotationVelocity;
    }

    public double getSetpoint() {
        return intakeConstants.intakeSetpoint;
    }

    public void setStopMode() {
        io.setStopMode();
    }

    public void setPlanetary(double speed) {
        speed = MathUtil.clamp(speed, -intakeConstants.IntakeMaxSpeed, intakeConstants.IntakeMaxSpeed);
        io.setPlanetary(speed);
    }

    public void setIntake(double speed) {
        io.setIntake(speed);
    }

    public void changeSetpoint(double setpoint) {
        intakeConstants.intakeSetpoint = setpoint;
    }

    public void stop() {
        io.setPlanetary(0);
        io.setIntake(0);
        isRollerActive = false;
    }

    public Command getToggleIntakeCommand() {
        return runOnce(() -> {
            if (isIntakeActive) {
                retract();
            } else {
                deploy();
            }
        });
    }

    public Command getToggleShootingMotionCommand() {
        return runOnce(() -> {
            enableShootingMotion = !enableShootingMotion;
        }).withName("ToggleShootingMotion");
    }

    public void IntakeShooting() {
        if (!enableShootingMotion) {
            changeSetpoint(isIntakeActive ? intakeConstants.CollectPosition : intakeConstants.StowedPosition);
            return;
        }

        if (intakeConstants.kIntakePushing == true && isFlipped == false) {
            changeSetpoint(intakeConstants.IntakeShootingPosition);
            isFlipped = true;

        } else if (intakeConstants.kIntakePushing == true && isFlipped == true) {
            changeSetpoint(isIntakeActive ? intakeConstants.CollectPosition : intakeConstants.StowedPosition);
            isFlipped = false;
        }
    }

    public Command getToggleRollersCommand() {
        return runOnce(() -> {
            if (isRollerActive) {
                setIntake(0);
                isRollerActive = false;
            } else {
                setIntake(intakeConstants.RollerSpeedCollect);
                isRollerActive = true;
            }
        });
    }

    public void deploy() {
        isIntakeActive = true;
        isRollerActive = true;
        intakeConstants.intakeCollecting = true;
        changeSetpoint(intakeConstants.CollectPosition);

        downController.reset(getPosition());

        setIntake(intakeConstants.RollerSpeedCollect);
    }

    public void retract() {
        isIntakeActive = false;
        isRollerActive = true;
        intakeConstants.intakeCollecting = true;

        changeSetpoint(intakeConstants.StowedPosition);

        upController.reset(getPosition());
    }

    public Command getToggleReverseRollersCommand() {
        return runOnce(() -> {

            if (isRollerReversed) {
                // Desliga reverse
                setIntake(0);
                isRollerReversed = false;

            } else {
                // Desativa modo normal
                isRollerActive = false;

                // Ativa reverse
                setIntake(-intakeConstants.RollerSpeedCollect);
                isRollerReversed = true;
            }

        }).withName("ToggleReverseRollers");
    }
}