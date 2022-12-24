// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SwerveDriveFieldXbox;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.controller.BetterXboxController;

public class RobotContainer {
  private final BetterXboxController driverController = new BetterXboxController(0, BetterXboxController.Hand.LEFT, BetterXboxController.Humans.DRIVER);
  private final BetterXboxController operatorController = new BetterXboxController(1, BetterXboxController.Humans.OPERATOR);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SendableChooser<Pose2d> positionChooser = new SendableChooser<>();

  public RobotContainer() {
    autoConfig();
    driverDashboardSetup();

    if (Constants.ROBOT_DEMO_MODE) {
      demoButtons();
    }
    else {
//      competitionButtons();
      testButtons();
    }

    Swerve.getInstance().setDefaultCommand(new SwerveDriveFieldXbox());
  }

  private void driverDashboardSetup() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  }

  private void testButtons() {}

  private void competitionButtons() {}

  private void demoButtons() {}

  private void autoConfig() {
    autoChooser.setDefaultOption("No auto", new WaitCommand(0));

    SmartDashboard.putData("Auto Command", autoChooser);
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}