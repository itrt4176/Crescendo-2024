package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double mainMotorOutput = 0.0;
        public double mainMotorCurrent = 0.0;

        public double followerMotorOutput = 0.0;
        public double followerMotorCurrent = 0.0;

        public double sharpSensorAverageVoltage = 0.0;
    }

    public default void updateInputs(IntakeInputs inputs) {
    }

    public default void setIntakeOutput(double percent) {
    }
}
