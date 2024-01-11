// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  public int gear = 3;
  public boolean running = false;

  CANSparkMax main = new CANSparkMax(ShooterConstants.kMainShooterMotor, MotorType.kBrushless);
  CANSparkMax sub = new CANSparkMax(ShooterConstants.kSubShooterMotor, MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    sub.follow(main, true);

    main.setIdleMode(CANSparkMax.IdleMode.kBrake);
    sub.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void shiftUp(){
    if(gear < 3){
      gear++;
    }
  }

  public void shiftDown(){
    if(gear > 0){
      gear--;
    }
  }

  public void run(){
    if(!running) {
    gear = 0;
    switch(gear){
      case 0:
      main.set(.25);
      break;
      case 1:
      main.set(.5);
      break;
      case 2:
      main.set(.75);
      break;
      case 3:
      main.set(1);
      break;
    }
  }
  else if(running) {
    main.set(0);
  }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
