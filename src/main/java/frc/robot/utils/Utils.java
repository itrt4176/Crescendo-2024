package frc.robot.utils;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;

public class Utils {
    public static Trajectory mapPPTrajectory(PathPlannerTrajectory ppTrajectory) {
        ArrayList<Trajectory.State> states = new ArrayList<>();

        for (PathPlannerTrajectory.State ppState : ppTrajectory.getStates()) {
            states.add(
                    new Trajectory.State(
                            ppState.timeSeconds,
                            ppState.velocityMps,
                            ppState.accelerationMpsSq,
                            new Pose2d(ppState.positionMeters, ppState.heading),
                            ppState.curvatureRadPerMeter));
        }
        
        return new Trajectory(states);
    }
}
