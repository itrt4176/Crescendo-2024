// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class SetClimberFlipper extends Command {
  /** Creates a new SetClimberFlipper. */
  private final Climber climber;
  private final double setpoint;
  private double error;

  public SetClimberFlipper(Climber climber, double setpoint) {
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
    error = setpoint - climber.getFlipDegrees();
    double speed = MathUtil.clamp(error * 0.025, -0.7, 0.7);

    if (Math.abs(speed) < 0.2) {
      return;
    }

    climber.setFlipSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setFlipSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) <= 0.5;
  }
}
