package frc.robot.subsystems.Intake;

public class IntakeConstants {
    public static final class intakeConstants {
        // ... (Mantenha seus IDs e outras constantes aqui) ...
        public static final int WheelMotorID = 32;
        public static final int RotationMotorID = 30;
        public static final double IntakeMaxSpeed = 0.7;
        public static final double RotationGearRatio = 25.0;

        // --- ADICIONE ESTAS LINHAS PARA CORRIGIR O ERRO ---
        
        // Velocidade suave para coletar automaticamente
        public static final double RollerSpeedCollect = 1.0;
        
        // Velocidade manual (mais forte se quiser)
        public static final double RollerSpeedManual = 0.5; 

        public static final double RollerSpeedEject = -0.5; 
        public static final double CollectPosition = 0.083; 
        public static final double StowedPosition = -0.020; 
        public static final double intakeMax = 0.10; 
        public static final double intakeMin = -0.04; 
        public static Boolean intakeCollecting = false;
        public static double intakeSetpoint = StowedPosition;
    }
}