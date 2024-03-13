package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterInputs {
        public double mainMotorOutput = 0.0;
        public double mainMotorCurrent = 0.0;

        public double followerMotorOutput = 0.0;
        public double followerMotorCurrent = 0.0;
    }

    public default void updateInputs(ShooterInputs inputs) {}

    public default void setShooterOutput(double percent) {}
}
