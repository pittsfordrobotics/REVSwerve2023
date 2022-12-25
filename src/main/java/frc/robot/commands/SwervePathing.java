package frc.robot.commands;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class SwervePathing extends SequentialCommandGroup {
    public SwervePathing(PathPlannerTrajectory trajectory, boolean reset) {
        super(
                new SwerveResetPose(trajectory, reset),
                new PPSwerveControllerCommand(
                        trajectory,
                        Swerve.getInstance()::getPose,
                        Swerve.getInstance().getKinematics(),
                        new PIDController(SwerveConstants.LINEAR_P, SwerveConstants.LINEAR_I, SwerveConstants.LINEAR_D),
                        new PIDController(SwerveConstants.LINEAR_P, SwerveConstants.LINEAR_I, SwerveConstants.LINEAR_D),
                        new PIDController(SwerveConstants.ROT_P, SwerveConstants.ROT_I, SwerveConstants.ROT_D),
                        Swerve.getInstance()::setModuleStates,
                        Swerve.getInstance()
                )
        );
    }
}