package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeIOReal implements IntakeIO {
    private final DutyCycleOut outputPercent = new DutyCycleOut(0);

    private TalonFX mainFx;
    private TalonFX followFx;
    private AnalogInput sharp;

    public IntakeIOReal() {
        mainFx = new TalonFX(INTAKE_MAIN);
        followFx = new TalonFX(INTAKE_FOLLOW);
        sharp = new AnalogInput(SHARP);

        mainFx.setNeutralMode(NeutralModeValue.Brake);
        followFx.setNeutralMode(NeutralModeValue.Brake);

        mainFx.setInverted(true);
        followFx.setInverted(true);

        followFx.setControl(new StrictFollower(mainFx.getDeviceID()));
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.mainMotorOutput = mainFx.get();
        inputs.mainMotorCurrent = mainFx.getTorqueCurrent().getValueAsDouble();

        inputs.followerMotorOutput = followFx.get();
        inputs.followerMotorCurrent = followFx.getTorqueCurrent().getValueAsDouble();

        inputs.sharpSensorAverageVoltage = sharp.getAverageVoltage();
    }

    @Override
    public void setIntakeOutput(double percent) {
        mainFx.setControl(outputPercent.withOutput(percent));
    }
}
