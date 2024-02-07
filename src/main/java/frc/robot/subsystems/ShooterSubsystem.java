// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax main = new CANSparkMax(ShooterConstants.MAIN_SHOOTER, MotorType.kBrushless);
  CANSparkMax sub = new CANSparkMax(ShooterConstants.SUB_SHOOTER, MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    main.setInverted(false);
    //sub.follow(main, true);
    sub.setInverted(true);

    main.setIdleMode(CANSparkMax.IdleMode.kCoast);
    sub.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void start(){
    setShootSpeed(.90);
  }

  public void stop() {
    setShootSpeed(0);
  }

  public void setShootSpeed(double speed) {
    main.set(speed);
    sub.set(speed);
  }

  public double getMainSpeed()
  {
    return main.get();
  }

  public double getSubSpeed() {
    return sub.get();
  }

  public void setDirectionalShootSpeed(double mainSpeed, double subSpeed) {
    main.set(mainSpeed);
    sub.set(subSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", main.getOutputCurrent());
  }
}
