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

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private boolean isIntakeActive = false;
    private boolean isRollerActive = false;

    private final ProfiledPIDController upController = new ProfiledPIDController(
        0.3, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(350, 300)
    );
    
    private final ProfiledPIDController downController = new ProfiledPIDController(
        0.5, 0.0, 0.00001, 
        new TrapezoidProfile.Constraints(350, 300)
    );

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

        SmartDashboard.putNumber("Intake/StowedPosition", intakeConstants.StowedPosition);
        SmartDashboard.putNumber("Intake/CollectPosition", intakeConstants.CollectPosition);
        SmartDashboard.putNumber("Intake/ABSOLUTE POSITION REAL", getPosition());
        SmartDashboard.putNumber("Intake/IntakeApplied Volts", inputs.rotationAppliedVolts);
        SmartDashboard.putNumber("Intake/IntakeCurrent Amps", inputs.rotationCurrentAmps);
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

    public void stop(){
        io.setPlanetary(0);
        io.setIntake(0);
        isRollerActive = false;
    }

    public Command getMaintainPositionCommand() {
        return run(() -> {
            double currentPos = getPosition();
            double target = getSetpoint();
            double output = 0;

            if (target > currentPos) {
                output = downController.calculate(currentPos, target);
            } else {
                output = upController.calculate(currentPos, target);
            }

            setPlanetary(output);
        });
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


    public Command getToggleRollersCommand() {
        return runOnce(() -> {
            if (isRollerActive) {
                setIntake(0);
                isRollerActive = false;
                System.out.println("Intake: ROLETES DESLIGADOS");
            } else {
                setIntake(intakeConstants.RollerSpeedCollect);
                isRollerActive = true;
                System.out.println("Intake: ROLETES LIGADOS");
            }
        });
    }

    private void deploy() {
        isIntakeActive = true;
        isRollerActive = true; // Sincroniza o estado do rolete
        intakeConstants.intakeCollecting = true;
        changeSetpoint(intakeConstants.CollectPosition);

        downController.reset(getPosition());

        setIntake(intakeConstants.RollerSpeedCollect); 
        
        System.out.println("Intake: DEPLOYED (Collecting)");
    }

    private void retract() {
        isIntakeActive = false;
        isRollerActive = false; // Sincroniza o estado do rolete
        intakeConstants.intakeCollecting = false;
        
        changeSetpoint(intakeConstants.StowedPosition); 
        
        // 2. Reseta o controlador
        upController.reset(getPosition());

        // 3. Desliga as rodas
        setIntake(0);
        
        System.out.println("Intake: RETRACTED (Stowed)");
    }

    // Comando Manual (Joystick)
    public Command getJoystickCommand(Supplier<Integer> pov, Supplier<Boolean> inBtn, Supplier<Boolean> outBtn) {
        return run(() -> {
            int povValue = pov.get();
            // Ajuste manual fino do setpoint
            if (povValue == OIConstants.kRaiseIntakeButtonIdx) {
                changeSetpoint(getSetpoint() - 0.005); // Pequeno ajuste pra cima
            } else if (povValue == OIConstants.kLowerIntakeButtonIdx) {
                changeSetpoint(getSetpoint() + 0.005); // Pequeno ajuste pra baixo
            }

            // Trava de segurança do Setpoint
            double clampedSetpoint = Math.max(intakeConstants.intakeMin, Math.min(getSetpoint(), intakeConstants.intakeMax));
            if(clampedSetpoint != getSetpoint()) changeSetpoint(clampedSetpoint);

            // Cálculo do PID Manual
            double currentPos = getPosition();
            double output = 0;
            if (getSetpoint() > currentPos) {
                output = upController.calculate(currentPos, getSetpoint());
            } else {
                output = downController.calculate(currentPos, getSetpoint());
            }
            setPlanetary(output);

            if (inBtn.get()) {
                setIntake(intakeConstants.RollerSpeedManual);
                isRollerActive = true;
            } else if (outBtn.get()) {
                setIntake(intakeConstants.RollerSpeedEject);
                isRollerActive = true;
            } else {
                if(!isIntakeActive){
                    setIntake(0);
                    isRollerActive = false;
                }
            }

        }).finallyDo(() -> {
            setStopMode();
            setIntake(0);
            isRollerActive = false;
        });
    }
}