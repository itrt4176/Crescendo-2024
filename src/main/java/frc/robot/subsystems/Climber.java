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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
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
  // private DigitalInput reverseLimitSwitch;

  // private AnalogInput homeSensor;
  private AnalogInput reverseSensor;

  public Climber() {

    flipperMain = new TalonFX(20);
    flipperFollow = new TalonFX(21);
    flipperFollow.setControl(new StrictFollower(flipperMain.getDeviceID()));
    // homeSensor = new AnalogInput(2);
    forwardLimitSwitch = new DigitalInput(FORWARD_LIMIT_DIO);
    reverseSensor = new AnalogInput(1);


    // winchFollow.follow(winchMain);
    flipperFollow.setInverted(true);

    TalonFXConfigurator configurator = flipperMain.getConfigurator();
    configurator.apply(new TalonFXConfiguration());//reset configs every time

    flipperOutput = new DutyCycleOut(0.0);
    flipperMain.setNeutralMode(NeutralModeValue.Brake);
    flipperFollow.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setFlipSpeed(double speed)
  {
    flipperMain.setControl(
      flipperOutput.withOutput(speed)
        // .withLimitForwardMotion(isHomed())
        // .withLimitReverseMotion(isFullyExtended())
    );
  }

  // FIX??? Might not need winch degrees.
  public double getFlipDegrees() 
  {
    return flipperMain.getPosition().getValueAsDouble() * ClimberConstants.FLIPPER_ROTATIONS_TO_DEGREES;
  }

  public boolean isHomed()
  {
    return !forwardLimitSwitch.get();
  }

public boolean isFullyExtended() {
  if(getReverseDistance() < 15.0) {
    return true;
  }
  return false;
}


  public void setZero()
  {
     flipperMain.setPosition(0);
     flipperFollow.setPosition(0);
  }

   //Private method, don't use outside of class because flipper neutral mode has to change :)


  //  public double getHomeDistance()
  // {
  //   return (Math.pow(homeSensor.getAverageVoltage(), -1.2045)) * 27.726;
  // }

  public double getReverseDistance() {
    return (Math.pow(reverseSensor.getAverageVoltage(), -1.2045)) * 27.726;
  }


  @Override
  public void periodic() {
  //Constantly checks the limit switches
  flipperMain.setControl(
    flipperOutput.withLimitForwardMotion(!forwardLimitSwitch.get())
      .withLimitReverseMotion(isFullyExtended())
  );
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flipper Degrees", flipperMain.getRotorPosition().getValueAsDouble() * ClimberConstants.FLIPPER_ROTATIONS_TO_DEGREES);
    // SmartDashboard.putNumber("Home Sensor Reading", getHomeDistance());
    SmartDashboard.putNumber("Reverse Sensor Reading", getReverseDistance());
    SmartDashboard.putBoolean("Switch", forwardLimitSwitch.get());

  }
}

