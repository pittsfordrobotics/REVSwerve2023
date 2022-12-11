// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.swerve.*;

import java.util.HashMap;

public final class Constants {
    /*  ROBOT  */
    public static final SwerveModuleIO ROBOT_FL_SWERVE_MODULE;
    public static final SwerveModuleIO ROBOT_FR_SWERVE_MODULE;
    public static final SwerveModuleIO ROBOT_BL_SWERVE_MODULE;
    public static final SwerveModuleIO ROBOT_BR_SWERVE_MODULE;
    public static final GyroIO ROBOT_GYRO;

    public static final boolean ROBOT_LOGGING_ENABLED = true;
    public static final String ROBOT_LOGGING_PATH = "/media/sda2/";
    public static final boolean ROBOT_PID_TUNER_ENABLED = false;
    public static final boolean ROBOT_DEMO_MODE = false;

    public static final HashMap<Integer, String> ROBOT_SPARKMAX_HASHMAP = new HashMap<>();
    static {
        ROBOT_SPARKMAX_HASHMAP.put(1, "Front Left Drive");
        ROBOT_SPARKMAX_HASHMAP.put(2, "Front Left Steer");
        ROBOT_SPARKMAX_HASHMAP.put(3, "Front Right Drive");
        ROBOT_SPARKMAX_HASHMAP.put(4, "Front Right Steer");
        ROBOT_SPARKMAX_HASHMAP.put(5, "Bottom Left Drive");
        ROBOT_SPARKMAX_HASHMAP.put(6, "Bottom Left Steer");
        ROBOT_SPARKMAX_HASHMAP.put(7, "Bottom Right Drive");
        ROBOT_SPARKMAX_HASHMAP.put(8, "Bottom Right Steer");
    }

    /*  DRIVE  */
    public static final int DRIVE_CAN_PIGEON = 0;
    public static final int DRIVE_CAN_FL_DRIVE = 1;
    public static final int DRIVE_CAN_FL_STEER = 2;
    public static final int DRIVE_CAN_FR_DRIVE = 3;
    public static final int DRIVE_CAN_FR_STEER = 4;
    public static final int DRIVE_CAN_BL_DRIVE = 5;
    public static final int DRIVE_CAN_BL_STEER = 6;
    public static final int DRIVE_CAN_BR_DRIVE = 7;
    public static final int DRIVE_CAN_BR_STEER = 8;

    public static final Pigeon2Configuration DRIVE_PIGEON_CONFIG = new Pigeon2Configuration();
    static {
        DRIVE_PIGEON_CONFIG.EnableCompass = false;
    }


    static {
        if (RobotBase.isReal() && ROBOT_DEMO_MODE) {
            ROBOT_FR_SWERVE_MODULE = new SwerveModuleIOSparkMax();
            ROBOT_FL_SWERVE_MODULE = new SwerveModuleIOSparkMax();
            ROBOT_BR_SWERVE_MODULE = new SwerveModuleIOSparkMax();
            ROBOT_BL_SWERVE_MODULE = new SwerveModuleIOSparkMax();
            ROBOT_GYRO = new GyroIOPigeon();
        }
        else if (RobotBase.isReal()) {
            ROBOT_FR_SWERVE_MODULE = new SwerveModuleIOSparkMax();
            ROBOT_FL_SWERVE_MODULE = new SwerveModuleIOSparkMax();
            ROBOT_BR_SWERVE_MODULE = new SwerveModuleIOSparkMax();
            ROBOT_BL_SWERVE_MODULE = new SwerveModuleIOSparkMax();
            ROBOT_GYRO = new GyroIOPigeon();
        }
        else {
            ROBOT_FR_SWERVE_MODULE = new SwerveModuleIOSim();
            ROBOT_FL_SWERVE_MODULE = new SwerveModuleIOSim();
            ROBOT_BR_SWERVE_MODULE = new SwerveModuleIOSim();
            ROBOT_BL_SWERVE_MODULE = new SwerveModuleIOSim();
            ROBOT_GYRO = new GyroIOSim();
        }
    }
}