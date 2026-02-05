package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        // Rotation Motor
        public double rotationPosition = 0.0;
        public double rotationVelocity = 0.0;
        public double rotationAppliedVolts = 0.0;
        public double rotationCurrentAmps = 0.0;

        // Wheel Motor
        public double wheelVelocity = 0.0;
        public double wheelAppliedVolts = 0.0;
        public double wheelCurrentAmps = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default double getPosition() { return 0.0; }

    public default double getSpeed() { return 0.0; }

    public default double getSetpoint() { return 0.0; }

    public default void setStopMode() {}

    public default void setPlanetary(double speed) {}

    public default void setIntake(double speed) {}

    public default void changeSetpoint(double setpoint) {}
}