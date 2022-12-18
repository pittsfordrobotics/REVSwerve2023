// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
        ROBOT_SPARKMAX_HASHMAP.put(5, "Back Left Drive");
        ROBOT_SPARKMAX_HASHMAP.put(6, "Back Left Steer");
        ROBOT_SPARKMAX_HASHMAP.put(7, "Back Right Drive");
        ROBOT_SPARKMAX_HASHMAP.put(8, "Back Right Steer");
    }

    /*  DRIVE: ALL IN METERS  */
    public static final int SWERVE_CAN_PIGEON = 0;
    public static final int SWERVE_CAN_FL_DRIVE = 1;
    public static final int SWERVE_CAN_FL_STEER = 2;
    public static final int SWERVE_CAN_FR_DRIVE = 3;
    public static final int SWERVE_CAN_FR_STEER = 4;
    public static final int SWERVE_CAN_BL_DRIVE = 5;
    public static final int SWERVE_CAN_BL_STEER = 6;
    public static final int SWERVE_CAN_BR_DRIVE = 7;
    public static final int SWERVE_CAN_BR_STEER = 8;

    public static final int SWERVE_THROUGH_BORE_COUNTS_PER_REV = 8192;
    public static final double SWERVE_WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double SWERVE_DRIVE_GEAR_RATIO = 5.08;
    public static final double SWERVE_STEER_GEAR_RATIO = 46.2962962963;

    public static final double SWERVE_X_LENGTH = Units.inchesToMeters(30);
    public static final double SWERVE_Y_LENGTH = Units.inchesToMeters(30);

    public static final Pigeon2Configuration SWERVE_PIGEON_CONFIG = new Pigeon2Configuration();
    static {
        SWERVE_PIGEON_CONFIG.EnableCompass = false;
    }

    public static final Rotation2d SWERVE_FL_OFFSET = Rotation2d.fromDegrees(0);
    public static final Rotation2d SWERVE_FR_OFFSET = Rotation2d.fromDegrees(0);
    public static final Rotation2d SWERVE_BL_OFFSET = Rotation2d.fromDegrees(0);
    public static final Rotation2d SWERVE_BR_OFFSET = Rotation2d.fromDegrees(0);

    public static final Translation2d[] SWERVE_MODULE_OFFSETS = {
        new Translation2d(-SWERVE_X_LENGTH / 2, SWERVE_Y_LENGTH / 2),
        new Translation2d(SWERVE_X_LENGTH / 2, SWERVE_Y_LENGTH / 2),
        new Translation2d(-SWERVE_X_LENGTH / 2, -SWERVE_Y_LENGTH / 2),
        new Translation2d(SWERVE_X_LENGTH / 2, -SWERVE_Y_LENGTH / 2),
    };

    static {
        if (RobotBase.isReal() && ROBOT_DEMO_MODE) {
            ROBOT_FL_SWERVE_MODULE = new SwerveModuleIOSparkMax(SWERVE_CAN_FL_DRIVE, SWERVE_CAN_FL_STEER, SWERVE_FL_OFFSET);
            ROBOT_FR_SWERVE_MODULE = new SwerveModuleIOSparkMax(SWERVE_CAN_FR_DRIVE, SWERVE_CAN_FR_STEER, SWERVE_FR_OFFSET);
            ROBOT_BL_SWERVE_MODULE = new SwerveModuleIOSparkMax(SWERVE_CAN_BL_DRIVE, SWERVE_CAN_BL_STEER, SWERVE_BL_OFFSET);
            ROBOT_BR_SWERVE_MODULE = new SwerveModuleIOSparkMax(SWERVE_CAN_BR_DRIVE, SWERVE_CAN_BR_STEER, SWERVE_BR_OFFSET);
            ROBOT_GYRO = new GyroIOPigeon();
        }
        else if (RobotBase.isReal()) {
            ROBOT_FL_SWERVE_MODULE = new SwerveModuleIOSparkMax(SWERVE_CAN_FL_DRIVE, SWERVE_CAN_FL_STEER, SWERVE_FL_OFFSET);
            ROBOT_FR_SWERVE_MODULE = new SwerveModuleIOSparkMax(SWERVE_CAN_FR_DRIVE, SWERVE_CAN_FR_STEER, SWERVE_FR_OFFSET);
            ROBOT_BL_SWERVE_MODULE = new SwerveModuleIOSparkMax(SWERVE_CAN_BL_DRIVE, SWERVE_CAN_BL_STEER, SWERVE_BL_OFFSET);
            ROBOT_BR_SWERVE_MODULE = new SwerveModuleIOSparkMax(SWERVE_CAN_BR_DRIVE, SWERVE_CAN_BR_STEER, SWERVE_BR_OFFSET);
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