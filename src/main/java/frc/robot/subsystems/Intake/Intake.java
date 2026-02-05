package frc.robot.subsystems.Intake;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake.IntakeConstants.intakeConstants;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final ProfiledPIDController upController = new ProfiledPIDController(
        0.08, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(200, 150)
    );
    
    private final ProfiledPIDController downController = new ProfiledPIDController(
        0.02, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(150, 100)
    );

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        SmartDashboard.putNumber("Intake Position", getPosition());
        SmartDashboard.putNumber("Intake Speed", getSpeed());
        SmartDashboard.putNumber("Intake Setpoint", getSetpoint());
    }


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

    public Command getJoystickCommand(Supplier<Integer> pov, Supplier<Boolean> inBtn, Supplier<Boolean> outBtn) {
        return run(() -> {
            int povValue = pov.get();
            if (povValue == OIConstants.kRaiseIntakeButtonIdx) {
                changeSetpoint(getSetpoint() + 0.2);
            } else if (povValue == OIConstants.kLowerIntakeButtonIdx) {
                changeSetpoint(getSetpoint() - 0.1);
            }

            // 2. Limita Setpoint
            double clampedSetpoint = Math.max(intakeConstants.intakeMin, Math.min(getSetpoint(), intakeConstants.intakeMax));
            changeSetpoint(clampedSetpoint);

            double currentPos = getPosition();
            double output = 0;

            if (getSetpoint() > currentPos) {
                output = upController.calculate(currentPos, getSetpoint());
            } else {
                output = downController.calculate(currentPos, getSetpoint());
            }

            setPlanetary(output);

            if (inBtn.get()) {
                setIntake(0.5);
            } else if (outBtn.get()) {
                setIntake(-0.5);
            } else {
                setIntake(0);
            }

        }).finallyDo(() -> {
            setStopMode();
            setIntake(0);
        });
    }

    public Command getCollectCommand(Supplier<Boolean> cancelButton) {
        return runOnce(() -> {
            changeSetpoint(intakeConstants.CollectPosition);
            setIntake(0.35);
            intakeConstants.intakeCollecting = true;
            System.out.println("Collect Initialized");
        })
        .andThen(
            run(() -> {
                double output = downController.calculate(getPosition(), intakeConstants.CollectPosition);
                setPlanetary(output);
            })
            .until(() -> cancelButton.get())
        )
        .finallyDo(() -> {
            System.out.println("Collect Ended");
            intakeConstants.intakeCollecting = false;
            setIntake(0);
            changeSetpoint(intakeConstants.intakeMax);
            setStopMode();
        });
    }
}