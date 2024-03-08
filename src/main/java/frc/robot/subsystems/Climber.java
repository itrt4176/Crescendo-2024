// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  final private DutyCycleOut flipperOutput; // This is nonsense
  private TalonFX flipperMain;
  private TalonFX flipperFollow;

  private DigitalInput forwardLimitSwitch;
  private DigitalInput reverseLimitSwitch;

  public Climber() {

    flipperMain = new TalonFX(20);
    flipperFollow = new TalonFX(21);
    flipperFollow.setControl(new StrictFollower(flipperMain.getDeviceID()));

    // winchFollow.follow(winchMain);
    flipperFollow.setInverted(true);

    TalonFXConfigurator configurator = flipperMain.getConfigurator();
    configurator.apply(new TalonFXConfiguration());//reset configs every time

    flipperOutput = new DutyCycleOut(0.0);
    flipperMain.setNeutralMode(NeutralModeValue.Brake);
    flipperFollow.setNeutralMode(NeutralModeValue.Brake);

    forwardLimitSwitch = new DigitalInput(FORWARD_LIMIT_DIO);
    reverseLimitSwitch = new DigitalInput(REVERSE_LIMIT_DIO);
  }

  public Trigger getForwardLimitSwitch() {
    return new Trigger(() -> !forwardLimitSwitch.get());
  }

  public Trigger getReverseLimitSwitch() {
    return new Trigger(() -> !reverseLimitSwitch.get());
  }

  public void setFlipSpeed(double speed)
  {
    flipperMain.setControl(
      flipperOutput.withOutput(speed)
        .withLimitForwardMotion(!forwardLimitSwitch.get())
        .withLimitReverseMotion(!reverseLimitSwitch.get())
    );
  }

  // FIX??? Might not need winch degrees.
  public double getFlipDegrees() 
  {
    return flipperMain.getPosition().getValueAsDouble() * ClimberConstants.FLIPPER_ROTATIONS_TO_DEGREES;
  }

   //Private method, don't use outside of class because flipper neutral mode has to change :)




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flipper Degrees", flipperMain.getRotorPosition().getValueAsDouble() * ClimberConstants.FLIPPER_ROTATIONS_TO_DEGREES);
  }
}
