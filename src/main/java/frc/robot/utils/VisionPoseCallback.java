package frc.robot.utils;

import java.util.Optional;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Represents a callback that accepts one {@link Pose2d} representing the
 * {@link SwerveSubsystem SwerveSubsystem's} estimated robot pose and produces an
 * {@link Optional}<{@link EstimatedRobotPose}> representing the
 * {@link VisionSubsystem VisionSubsystem's} estimate of the current robot pose.
 *
 * <p>
 * This is a {@link FunctionalInterface functional interface}
 * whose functional method is {@link #apply(Pose2d)}.
 */
@FunctionalInterface
public interface VisionPoseCallback extends Function<Pose2d, Optional<EstimatedRobotPose>> {
    @Override
    public Optional<EstimatedRobotPose> apply(Pose2d drivePoseEstimate);
}
