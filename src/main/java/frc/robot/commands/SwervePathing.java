package frc.robot.commands;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwervePathing extends SequentialCommandGroup {
    public SwervePathing(Trajectory trajectory, boolean resetPose) {
        super(
                new SwerveZero(),
                new SwerveResetPose(trajectory, resetPose)
//                new PPSwerveControllerCommand(
//                        trajectory,
//                        Swerve.getInstance().getPose(),
//                        Swerve.getInstance().getKinematics(),
//
//                        Swerve.getInstance()::setModuleStates(),
//                        Swerve.getInstance()
//                ),
        );
    }
}