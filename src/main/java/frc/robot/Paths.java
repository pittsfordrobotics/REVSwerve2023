package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.Constants.SwerveConstants;

import java.util.ArrayList;

public class Paths {
    public static final PathPlannerTrajectory TEST = PathPlanner.loadPath("Test", SwerveConstants.MAX_SPEED);
    public static final ArrayList<PathPlannerTrajectory> FIVE_BALL = PathPlanner.loadPathGroup("5 Ball", SwerveConstants.MAX_SPEED);
}