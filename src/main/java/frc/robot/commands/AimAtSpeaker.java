// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AimAtSpeaker extends Command {
  /** Creates a new AimAtSpeaker. */
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private int speakerID;
  private PhotonTrackedTarget target;
  private double error;

  public AimAtSpeaker(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PhotonPipelineResult result = vision.getLatestResult();
      
      int speakerID = 7; //defaults to blue alliance
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            speakerID = 4;
        }
        if (ally.get() == Alliance.Blue) {
            speakerID = 7;
        }
      }

      // Query the latest result from PhotonVision
      if (result.hasTargets()) {
          // Calculate angular turn power
          // -1.0 required to ensure positive PID controller effort _increases_ yaw
          for(PhotonTrackedTarget target : result.targets) {
            if (target.getFiducialId() == speakerID) {
              this.target = target;
            }
          }
      } else {
        end(false);
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = target.getYaw();

    double rotationSpeed = 0;

    swerve.driveCommand(
      () -> 0,
      () -> 0,
      () -> rotationSpeed
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.driveCommand(
      () -> 0,
      () -> 0,
      () -> 0
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) <= .1;
  }
}
