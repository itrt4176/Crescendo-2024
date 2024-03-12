// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.*;

public class Shoot extends Command {

  private final ShooterSubsystem shooter;
  private final Intake intake;
  private final double speed;
  //add climber later

  /** Creates a new SpeakerShoot. */
  public Shoot(ShooterSubsystem shooter, Intake intake, double speed) {
    this.shooter = shooter;
    this.intake = intake;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shooter.setShootSpeed(speed);
      
      // intake.setIntakeSpeed(.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("execute");
    MedianFilter filter = new MedianFilter(25);
    if (filter.calculate(shooter.getSpeed()) >= speed - 0.005);
    { 
      intake.setIntakeSpeed(-.3);
      System.out.println("up to speed");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    // shooter.setShootSpeed(0);
    // intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake.isNoteLoaded();
    // return false;
  }
}
