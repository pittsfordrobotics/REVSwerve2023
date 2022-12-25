package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;


public class SwerveZero extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();

    public SwerveZero() {
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerve.driveZero();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}