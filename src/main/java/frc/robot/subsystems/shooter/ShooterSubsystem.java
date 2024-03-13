// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  public boolean running = false;
  private final ShooterIO io;
  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void setShootSpeed(double speed) {
    running = speed != 0.0;
    io.setShooterOutput(speed);
  }

  public double getSpeed() {
    return inputs.mainMotorOutput;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    SmartDashboard.putNumber("Shooter Current", inputs.mainMotorCurrent);
    SmartDashboard.putNumber("Shooter Speed", getSpeed());
  }
}
