// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  public int gear = 0;
  public boolean running = false;

  CANSparkMax main = new CANSparkMax(ShooterConstants.MAIN_SHOOTER, MotorType.kBrushless);
  CANSparkMax sub = new CANSparkMax(ShooterConstants.SUB_SHOOTER, MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    main.setInverted(false);
    sub.follow(main, true);

    main.setIdleMode(CANSparkMax.IdleMode.kBrake);
    sub.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /**
   * Increases motor speed and starts the motor if it is not running.
   */
  public void shiftUp(){
    // if(gear < 3 && running){
    //   gear++;
    //   start();
    // }
    if(gear < 3){
      gear++;
      switchGear(gear);
    }
  }

  /**
   * Decreases motor speed and starts the motor
   */
  public void shiftDown(){
    // if(gear > 0 && running){
    //   gear--;
    //   start();
    // }
    if(gear > 0){
      gear--;
      switchGear(gear);
    }
  }

  public void start(){
    // if (running == false) {
    //   gear = 0;
    //   running = true;
    //   switchGear(gear);
    // }

    setShootSpeed(.75);
  }

  public void stop() {
    running = false;
    main.setIdleMode(CANSparkMax.IdleMode.kCoast);
    sub.setIdleMode(CANSparkMax.IdleMode.kCoast);
    main.set(0);

  }

  private void switchGear(int gear) {
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



  public void setShootSpeed(double speed) {
    running = true;
    main.setIdleMode(CANSparkMax.IdleMode.kBrake);
    sub.setIdleMode(CANSparkMax.IdleMode.kBrake);
    main.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
