package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.controller.BetterXboxController;
import frc.robot.util.controller.BetterXboxController.Humans;


public class SwerveDriveFieldXbox extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();

    public SwerveDriveFieldXbox() {
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        swerve.driveFieldOrientated(
            -BetterXboxController.getController(Humans.DRIVER).getLeftY() * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND,
            -BetterXboxController.getController(Humans.DRIVER).getLeftX() * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND,
            BetterXboxController.getController(Humans.DRIVER).getRightX() * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}