// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import static frc.robot.Constants.VisionConstants.*;
import static java.util.Objects.requireNonNull;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private boolean enabled  = false;

  private static VisionSubsystem instance = null;

  private final static AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final static PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  /**
   * 3D position of the camera on the robot according to the <a
   * href=
   * "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">WPILib
   * Coordinate System</a>.
   * 
   * @see VisionConstants#TRANS_X
   * @see VisionConstants#TRANS_Y
   * @see VisionConstants#TRANS_Z
   * @see VisionConstants#PITCH
   * @see VisionConstants#ROLL
   * @see VisionConstants#YAW
   */
  private final static Transform3d robotToCam = new Transform3d(
      new Translation3d(TRANS_X, TRANS_Y, TRANS_Z),
      new Rotation3d(ROLL, PITCH, YAW)
  );
  
  private PhotonCamera limelight;
  private PhotonPoseEstimator poseEstimator;

  private Supplier<Pose2d> simPoseSupplier = null;
  private VisionSystemSim visionSim;

  /** Creates a new VisionSubsystem. */
  private VisionSubsystem() {
    limelight = new PhotonCamera(LIMELIGHT_NAME);
    limelight.setDriverMode(!enabled);
    limelight.setLED(VisionLEDMode.kOff);

    poseEstimator = new PhotonPoseEstimator(
      fieldLayout,
      poseStrategy,
      limelight,
      robotToCam
    );  

    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(fieldLayout);

      var simLimelightProps = SimCameraProperties.LL2_960_720();
      var limelightSim = new PhotonCameraSim(limelight, simLimelightProps);
      limelightSim.enableDrawWireframe(true);

      visionSim.addCamera(limelightSim, robotToCam);
    }
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

  public Optional<EstimatedRobotPose> getLimelightEstimatedPose(Pose2d currentPoseEstimate) {
    if (enabled) {
      poseEstimator.setReferencePose(currentPoseEstimate);
      return poseEstimator.update();
    }

    return Optional.empty();
  }

  public void setSimPoseSupplier(Supplier<Pose2d> poseSupplier) {
    simPoseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    requireNonNull(simPoseSupplier,
        () -> "simPoseSupplier must not be null! Use VisionSystem.setSimPoseSupplier(Supplier<Pose2d>) to resolve this.");
    visionSim.update(simPoseSupplier.get());
    SmartDashboard.putData("VisionSim Field", visionSim.getDebugField());
  }
}
