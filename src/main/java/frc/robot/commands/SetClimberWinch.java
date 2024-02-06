// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class SetClimberWinch extends Command {

  private final Climber climber;
  private final double setpoint;
  private double error;

  /** Creates a new SetClimberWinch. */
  public SetClimberWinch(Climber climber, double setpoint) {
    this.climber = climber;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = setpoint - climber.getWinchDegrees();
    double speed = MathUtil.clamp(error * 0.025, -0.7, 0.7);

    if (Math.abs(speed) < 0.2) {
      return;
    }

    climber.setWinchSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
