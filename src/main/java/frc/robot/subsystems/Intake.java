// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.AnalogTriggerOutput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */


  private TalonFX mainFx;
  private TalonFX followFx;
  private AnalogInput sharp;

  private Servo rotateCam;
  private Servo arcCam;

  public Intake() {
    mainFx = new TalonFX(Constants.IntakeConstants.INTAKE_MAIN);
    followFx = new TalonFX(Constants.IntakeConstants.INTAKE_FOLLOW);
    sharp = new AnalogInput(Constants.IntakeConstants.SHARP);

    mainFx.setNeutralMode(NeutralModeValue.Brake);
    followFx.setNeutralMode(NeutralModeValue.Brake);

    mainFx.setInverted(true);
    followFx.setInverted(true);

    rotateCam = new Servo(0);
    arcCam = new Servo(1);
    
  
  }


  public void setIntakeSpeed(double speed)
  {
    mainFx.set(speed);
    followFx.set(speed);//to make up for lack of follow method now

  }

  public void on()
  {
    setIntakeSpeed(-.3);
  }

  public void reverse()
  {
    setIntakeSpeed(.2);
  }

  public void stop()
  {
    mainFx.set(0);
    followFx.set(0);
  }


  public double getDistance()
  {
    return (Math.pow(sharp.getAverageVoltage(), -1.2045)) * 27.726;
  }

  public boolean isNoteLoaded()
  {
    if(getDistance() < 40.0)//placeholder condition that needs to be tested
    {
      return true;
    }

    return false;
  }

  public void lookIntake()
  {
    rotateCam.setPosition(.1);
    arcCam.setPosition(.5);
  }

  public void lookClimber()
  {
    rotateCam.setPosition(0.6);
    arcCam.setPosition(.35);
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Sensor Reading", getDistance());
    SmartDashboard.putBoolean("isNoteLoaded", isNoteLoaded());
    SmartDashboard.putNumber("Intake Current", mainFx.getTorqueCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Arc Cam Pos", arcCam.getPosition());
    SmartDashboard.putNumber("Rotate Cam Pos", rotateCam.getPosition());
  }
}
