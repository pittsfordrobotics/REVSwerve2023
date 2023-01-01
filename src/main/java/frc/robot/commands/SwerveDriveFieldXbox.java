package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
//        bc we have bad controller that will drift and prolly screw up auto
        if (!DriverStation.isAutonomous()) {
            swerve.driveFieldOrientated(
                    BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getX(),
                    BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getY(),
                    BetterXboxController.getController(Humans.DRIVER).getSwerveRotation()
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}