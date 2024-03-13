// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public void setIntakeSpeed(double speed) {
    io.setIntakeOutput(speed);
  }

  public void on() {
    setIntakeSpeed(-.3);
  }

  public void reverse() {
    setIntakeSpeed(.2);
  }

  public void stop() {
    setIntakeSpeed(0);
  }

  public double getDistance() {
    return (Math.pow(inputs.sharpSensorAverageVoltage, -1.2045)) * 27.726;
  }

  public boolean isNoteLoaded() {
    if (getDistance() < 30.0)// placeholder condition that needs to be tested
    {
      return true;
    }

    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    SmartDashboard.putNumber("Sensor Reading", getDistance());
    SmartDashboard.putBoolean("isNoteLoaded", isNoteLoaded());
    SmartDashboard.putNumber("Intake Current", inputs.mainMotorCurrent);
  }
}
