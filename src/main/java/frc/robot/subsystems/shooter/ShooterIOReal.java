package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterIOReal implements ShooterIO {
    private CANSparkFlex main;
    private CANSparkFlex follower;

    public ShooterIOReal() {
        main = new CANSparkFlex(MAIN_SHOOTER, MotorType.kBrushless);
        main.setInverted(false);
        main.setIdleMode(CANSparkFlex.IdleMode.kCoast);
        // main.enableVoltageCompensation(12.0);

        follower = new CANSparkFlex(SUB_SHOOTER, MotorType.kBrushless);
        follower.follow(main, true);
        follower.setIdleMode(CANSparkFlex.IdleMode.kCoast);
        // follower.enableVoltageCompensation(12.0);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.mainMotorOutput = main.get();
        inputs.mainMotorCurrent = main.getOutputCurrent();
        inputs.followerMotorOutput = follower.get();
        inputs.followerMotorCurrent = follower.getOutputCurrent();
    }

    @Override
    public void setShooterOutput(double percent) {
        main.set(percent);
    }

}
