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

    private boolean isIntakeActive = false;

    private final ProfiledPIDController upController = new ProfiledPIDController(
        0.08, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(300, 150)
    );
    
    private final ProfiledPIDController downController = new ProfiledPIDController(
        0.02, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(150, 100)
    );

public Intake(IntakeIO io) {
        this.io = io;
        intakeConstants.intakeSetpoint = intakeConstants.StowedPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        SmartDashboard.putNumber("Intake/Position", getPosition());
        SmartDashboard.putNumber("Intake/Speed", getSpeed());
        SmartDashboard.putNumber("Intake/Setpoint", getSetpoint());
        SmartDashboard.putBoolean("Intake/Is Active", isIntakeActive);
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

    // --- SETTERS ---
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
    }

    // =========================================================================
    //  LÓGICA DE COMANDOS
    // =========================================================================

    /**
     * COMANDO PADRÃO (Default Command)
     * Roda sempre. Mantém o braço na posição do Setpoint usando PID.
     */
    public Command getMaintainPositionCommand() {
        return run(() -> {
            double currentPos = getPosition();
            double target = getSetpoint();
            double output = 0;

            // Se o alvo é maior (mais pra baixo) que o atual -> DESCER
            if (target > currentPos) {
                output = downController.calculate(currentPos, target);
            } else {
                // Se o alvo é menor (mais pra cima) -> SUBIR
                output = upController.calculate(currentPos, target);
            }

            setPlanetary(output);
        });
    }

    /**
     * COMANDO TOGGLE (Alternar)
     */
    public Command getToggleIntakeCommand() {
        return runOnce(() -> {
            if (isIntakeActive) {
                retract();
            } else {
                deploy();
            }
        });
    }

    private void deploy() {
        isIntakeActive = true;
        intakeConstants.intakeCollecting = true;
        
        // 1. Muda o Setpoint para posição de coleta (ex: 0.083)
        changeSetpoint(intakeConstants.CollectPosition);
        
        // 2. Reseta o controlador para evitar "pulo"
        downController.reset(getPosition());

        setIntake(intakeConstants.RollerSpeedCollect); 
        
        System.out.println("Intake: DEPLOYED (Collecting)");
    }

    private void retract() {
        isIntakeActive = false;
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

            // Controle das Rodas Manual
            if (inBtn.get()) {
                setIntake(intakeConstants.RollerSpeedManual);
            } else if (outBtn.get()) {
                setIntake(intakeConstants.RollerSpeedEject);
            } else {
                setIntake(0);
            }

        }).finallyDo(() -> {
            setStopMode();
            setIntake(0);
        });
    }
}