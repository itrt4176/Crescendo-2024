// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  TalonFX flipper;

  public Climber() {

    flipper = new TalonFX(16);

    flipper.setNeutralMode(NeutralModeValue.Brake);
  }


  public void setSpeed(double speed)
  {
    flipper.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
