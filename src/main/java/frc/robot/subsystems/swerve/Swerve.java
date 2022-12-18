package frc.robot.subsystems.swerve;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    /*
     * Swerve Module Orientation
     *        FL  FR
     *
     *        BL  BR
     */
    private final SwerveModuleIO[] moduleIO;
    private final SwerveModuleIOInputsAutoLogged[] moduleInputs = {};

    public enum Modules {
        FRONT_LEFT(0), FRONT_RIGHT(1), BOTTOM_LEFT(2), BOTTOM_RIGHT(3);

        final int id;
        Modules(int id){
            this.id = id;
        }
        public int getId() {
            return id;
        }
    }

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(Constants.SWERVE_MODULE_OFFSETS);
    private final SwerveModulePosition[] modulePositions = {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
    };
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(driveKinematics, new Rotation2d(0), modulePositions);

    private final static Swerve INSTANCE = new Swerve(Constants.ROBOT_FL_SWERVE_MODULE, Constants.ROBOT_FR_SWERVE_MODULE, Constants.ROBOT_BL_SWERVE_MODULE, Constants.ROBOT_BR_SWERVE_MODULE, Constants.ROBOT_GYRO);

    public static Swerve getInstance() {
        return INSTANCE;
    }

    private Swerve(SwerveModuleIO FL, SwerveModuleIO FR, SwerveModuleIO BL, SwerveModuleIO BR, GyroIO gyro) {
        moduleIO = new SwerveModuleIO[]{FL, FR, BL, BR};
        gyroIO = gyro;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
        }
        Logger.getInstance().processInputs("Front Left Swerve Module", moduleInputs[0]);
        Logger.getInstance().processInputs("Front Right Swerve Module", moduleInputs[1]);
        Logger.getInstance().processInputs("Back Left Swerve Module", moduleInputs[2]);
        Logger.getInstance().processInputs("Back Right Swerve Module", moduleInputs[3]);

        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Gyro", gyroInputs);
    }
}