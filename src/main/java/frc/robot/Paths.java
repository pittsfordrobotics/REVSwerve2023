package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.Constants.SwerveConstants;

public class Paths {
    public static final PathPlannerTrajectory test = PathPlanner.loadPath("Test", SwerveConstants.MAX_SPEED);
}