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
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj.TimedRobot;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  final private DutyCycleOut flipperOutput; // This is nonsense
  private TalonFX flipperMain;
  private TalonFX flipperFollow;

  private DigitalInput forwardLimitSwitch;
  private AnalogInput reverseSensor;

  private DutyCycleEncoder encoder;

  private boolean encoderForwardLimitReached = false;
  private boolean encoderReverseLimitReached = false; 

  public Climber() {

    flipperMain = new TalonFX(20);
    flipperFollow = new TalonFX(21);
    flipperFollow.setControl(new StrictFollower(flipperMain.getDeviceID()));
    forwardLimitSwitch = new DigitalInput(FORWARD_LIMIT_DIO);
    reverseSensor = new AnalogInput(1);

    encoder = new DutyCycleEncoder(7);
    encoder.setPositionOffset(encoderOffset);

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
    );
  }

  public double getFlipDegrees() 
  {
    return flipperMain.getPosition().getValueAsDouble() * ClimberConstants.FLIPPER_ROTATIONS_TO_DEGREES;
  }

  public boolean forwardLimitSwitchReached() //forward is home
  {
    return !forwardLimitSwitch.get();
  }

  public boolean reverseLimitSwitchReached() {
    if(getReverseDistance() < 15.0) {
      return true;
    }
    return false;
  }

  public boolean encoderForwardLimitReached() {return encoderForwardLimitReached;}
  public boolean encoderReverseLimitReached() {return encoderReverseLimitReached;}


  public boolean forwardCombinedLimit() {
    return forwardLimitSwitchReached() || encoderForwardLimitReached();
  }

  public boolean reverseCombinedLimit() {
    return reverseLimitSwitchReached() || encoderReverseLimitReached();
  }

  public void zeroMotors()
  {
     flipperMain.setPosition(0);
     flipperFollow.setPosition(0);
  }

  public double getReverseDistance() {
    return (Math.pow(reverseSensor.getAverageVoltage(), -1.2045)) * 27.726;
  }

  //do not use outside of class
  private double getRawEncoderDegrees() {
    return encoder.getDistance() * 360;
  }

  //Only accounts for one revolution of wrapping in either direction
  public double getEncoderDegrees() {
    double wrapAdjustment = 0.0;
    if(getRawEncoderDegrees() > rawEncoderDegCalcUB) {
      wrapAdjustment = -1.0;
    } else if(getRawEncoderDegrees() < rawEncoderDegCalcLB) {
      wrapAdjustment = 1.0;
    }
    return (encoder.getDistance() + wrapAdjustment) * -360;
  }

  @Override
  public void periodic() {
  //Constantly checks the limit switches
  encoderReverseLimitReached = getEncoderDegrees() >= reverseEncoderLimitDegrees;
  encoderForwardLimitReached = getEncoderDegrees() <= 0;
  
  flipperMain.setControl(
    flipperOutput.withLimitForwardMotion(forwardCombinedLimit())
      .withLimitReverseMotion(reverseCombinedLimit())
  );
  
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flipper Degrees", flipperMain.getRotorPosition().getValueAsDouble() * ClimberConstants.FLIPPER_ROTATIONS_TO_DEGREES);
    // SmartDashboard.putNumber("Home Sensor Reading", getHomeDistance());
    SmartDashboard.putNumber("Reverse Sensor Reading", getReverseDistance());
    SmartDashboard.putBoolean("Switch", forwardLimitSwitch.get());
    SmartDashboard.putNumber("Encoder Degrees", (getEncoderDegrees()));
    SmartDashboard.putNumber("Unfiltered Encoder Degrees", (encoder.getDistance() * 360));
    SmartDashboard.putNumber("Absolute Encoder Degrees", encoder.getAbsolutePosition() * 360);
    SmartDashboard.putNumber("Absolute Encoder", encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Distance", encoder.getDistance());


  }

  
}

