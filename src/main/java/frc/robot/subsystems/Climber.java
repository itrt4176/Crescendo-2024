// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ClimberConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {

  public int gear = 0;
  public boolean running = false;

  CANSparkMax main = new CANSparkMax(ClimberConstants.Main_Climber, MotorType.kBrushless);
  CANSparkMax sub = new CANSparkMax(ClimberConstants.Sub_Climber, MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public Climber() {

    main.setInverted(false);
    sub.follow(main, true);

    main.setIdleMode(CANSparkMax.IdleMode.kBrake);
    sub.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /**
   * Increases motor speed and starts the motor if it is not running.
   */


  public void start(){
    if (running == false) {
      running = true;
      main.set(.1);
    }
  }

  public void stop() {
    running = false;
    main.setIdleMode(CANSparkMax.IdleMode.kCoast);
    sub.setIdleMode(CANSparkMax.IdleMode.kCoast);
    main.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
