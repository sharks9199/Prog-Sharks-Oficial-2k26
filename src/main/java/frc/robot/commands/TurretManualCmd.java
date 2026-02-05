package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class TurretManualCmd extends Command {
    
    private final Shooter shooter;
    private final Supplier<Integer> pov;

    private double currentTurretSetpoint;
    private final double kTurretSpeed = 2.5;
    private final double kMinTurretAngle = -90.0;
    private final double kMaxTurretAngle = 90.0;

    private double currentPivotSetpoint;
    private final double kPivotSpeed = 5.5;
    

    private final double kMinPivotAngle = 0.0; 
    private final double kMaxPivotAngle = 80.0; 

    public TurretManualCmd(Shooter shooter, Supplier<Integer> pov) {
        this.shooter = shooter;
        this.pov = pov;
        
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        currentTurretSetpoint = shooter.getTurretPosition();
        currentPivotSetpoint = shooter.getPivotPosition(); 
    }

    @Override
    public void execute() {
        int povValue = pov.get();
        if (povValue == 270) { 
            currentTurretSetpoint -= kTurretSpeed;
        } 
        else if (povValue == 90) { 
            currentTurretSetpoint += kTurretSpeed;
        }

        if (povValue == 0) {
            currentPivotSetpoint += kPivotSpeed;
        }
        else if (povValue == 180) {
            currentPivotSetpoint -= kPivotSpeed;
        }

        // Aplica os limites (Clamp)
        currentTurretSetpoint = MathUtil.clamp(currentTurretSetpoint, kMinTurretAngle, kMaxTurretAngle);
        currentPivotSetpoint = MathUtil.clamp(currentPivotSetpoint, kMinPivotAngle, kMaxPivotAngle);

        // Envia para o Subsystem
        shooter.setTurretSetpoint(currentTurretSetpoint);
        shooter.setPivotPosition(currentPivotSetpoint);

        if (povValue != -1) {
            System.out.printf("PIVOT DEBUG >> Botão: %d | Alvo (Setpoint): %.2f | Posição Real: %.2f%n", 
                povValue, 
                currentPivotSetpoint, 
                shooter.getPivotPosition()
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}