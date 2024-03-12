// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

public class VisionSubsystem extends SubsystemBase {
  private boolean enabled = true;

  private static VisionSubsystem instance = null;

  private final static AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final static PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  /**
   * 3D position of the camera on the robot according to the <a
   * href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib Coordinate System</a>.
   * 
   * @see VisionConstants#transX
   * @see VisionConstants#transY
   * @see VisionConstants#transZ
   * @see VisionConstants#pitch
   * @see VisionConstants#roll
   * @see VisionConstants#yaw
   */
  private final static Transform3d robotToCam = new Transform3d(
    new Translation3d(transX, transY, transZ),
    new Rotation3d(roll, pitch, yaw)
  );
  
  private PhotonCamera limelight;
  private PhotonPoseEstimator poseEstimator;

  /** Creates a new VisionSubsystem. */
  private VisionSubsystem() {
    limelight = new PhotonCamera(limelightName);
    limelight.setDriverMode(false);
    limelight.setLED(VisionLEDMode.kOff);

    poseEstimator = new PhotonPoseEstimator(
      fieldLayout,
      poseStrategy,
      limelight,
      robotToCam
    );  
  }

  public static VisionSubsystem getInstance() {
    if (instance == null) {
      instance = new VisionSubsystem();
    }

    return instance;
  }

  public boolean isEnabled() {
    return enabled;
  }
  
  public void enable() {
    limelight.setDriverMode(false);
    enabled = true;
  }

  public void disable() {
    enabled = false;
    limelight.setDriverMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
