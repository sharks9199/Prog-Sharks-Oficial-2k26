package frc.robot.subsystems.intake;

public class IntakeConstants {
    public static final class intakeConstants {
        public static final int WheelMotorID = 32;
        public static final int RotationMotorID = 34;
        public static final double IntakeMaxSpeed = 0.7;

        public static final int ThroughBoreEncoderPort = 0;
        public static final double EncoderOffset = 0.0;

        public static final double RollerSpeedCollect = 2.0;
        public static final double RollerSpeedManual = 0.5; 

        public static final double RollerSpeedEject = -0.5; 
        public static final double CollectPosition = 0.535; 
        public static final double StowedPosition = 0.250; 
        
        public static final double intakeMax = 0.560;
        public static final double intakeMin = 0.24;
        public static Boolean intakeCollecting = false;
        public static double intakeSetpoint = StowedPosition;
    }
}