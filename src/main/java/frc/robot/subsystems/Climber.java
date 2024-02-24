// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  TalonFX flipper;

  CANSparkMax winchMain;
  CANSparkMax winchFollow;

  

  

  public Climber() {

    flipper = new TalonFX(16);
    winchMain = new CANSparkMax(25, MotorType.kBrushless);
    winchFollow = new CANSparkMax(26, MotorType.kBrushless);

    // winchFollow.follow(winchMain);
    winchFollow.setInverted(true);

    TalonFXConfigurator configurator = flipper.getConfigurator();
    configurator.apply(new TalonFXConfiguration());//reset configs every time

    HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();//configure limit switch to be able to set flipper to 0 degrees.
    limitConfigs.withForwardLimitAutosetPositionEnable(true)
      .withForwardLimitAutosetPositionValue(0.0)
      .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
      .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
      .withForwardLimitEnable(true);

    configurator.apply(limitConfigs);


    flipper.setNeutralMode(NeutralModeValue.Brake);
    winchMain.setIdleMode(IdleMode.kBrake);
    winchFollow.setIdleMode(IdleMode.kBrake);
  }


  public void setFlipSpeed(double speed)
  {
    flipper.set(speed);
  }

  // FIX??? Might not need winch degrees.
  public double getFlipDegrees() 
  {
    return flipper.getPosition().getValueAsDouble() * ClimberConstants.FLIPPER_ROTATIONS_TO_DEGREES;
  }

   //Private method, don't use outside of class because flipper neutral mode has to change :)
  private void setWinchSpeed(double speed)
  {
    winchMain.set(speed);
  }

  public void winchRetract()
  {
    setWinchSpeed(.1);
    flipper.setNeutralMode(NeutralModeValue.Coast);
  }

  public void winchReverse()
  {
    setWinchSpeed(-.1);
    flipper.setNeutralMode(NeutralModeValue.Coast);
  }

  public void stopWinch()
  {
    setWinchSpeed(0);
    flipper.setNeutralMode(NeutralModeValue.Brake);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flipper Degrees", flipper.getRotorPosition().getValueAsDouble() * ClimberConstants.FLIPPER_ROTATIONS_TO_DEGREES);
  }
}
