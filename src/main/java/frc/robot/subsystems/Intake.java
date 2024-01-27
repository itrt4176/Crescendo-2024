// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */


  private TalonFX mainFx;
  private TalonFX followFx;
  private AnalogInput sharp;

  public Intake() {
    mainFx = new TalonFX(Constants.IntakeConstants.INTAKE_MAIN);
    followFx = new TalonFX(Constants.IntakeConstants.INTAKE_FOLLOW);
    sharp = new AnalogInput(Constants.IntakeConstants.SHARP);

    mainFx.setNeutralMode(NeutralModeValue.Brake);
    followFx.setNeutralMode(NeutralModeValue.Brake);

    followFx.setInverted(true);
    
  }


  public void setIntakeSpeed(double speed)
  {
    mainFx.set(speed);
    followFx.set(mainFx.get());//to make up for lack of follow method now

  }

  public void on()
  {
    setIntakeSpeed(.4);
  }

  public void stop()
  {
    mainFx.set(0);
    followFx.set(mainFx.get());
  }


  public double getDistance()
  {
    return (Math.pow(sharp.getAverageVoltage(), -1.2045)) * 27.726;
  }

  public boolean isNoteLoaded()
  {
    if(getDistance() < 0)//placeholder condition that nedds to be tested
    {
      return true;
    }

    return false;
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
