package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.GeomUtil;
import frc.robot.util.SwerveOptimizer;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    /*
     * Swerve Module Orientation
     *        FL  FR
     *
     *        BL  BR
     */
    private final SwerveModuleIO[] moduleIO;
    private final SwerveModuleIOInputsAutoLogged[] moduleInputs = new SwerveModuleIOInputsAutoLogged[] {new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged()};

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
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private SwerveModuleState[] moduleStates = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    private final SwerveDrivePoseEstimator poseEstimator;

    private final static Swerve INSTANCE = new Swerve(Constants.ROBOT_FL_SWERVE_MODULE, Constants.ROBOT_FR_SWERVE_MODULE, Constants.ROBOT_BL_SWERVE_MODULE, Constants.ROBOT_BR_SWERVE_MODULE, Constants.ROBOT_GYRO);

    public static Swerve getInstance() {
        return INSTANCE;
    }

    private Swerve(SwerveModuleIO FL, SwerveModuleIO FR, SwerveModuleIO BL, SwerveModuleIO BR, GyroIO gyro) {
        moduleIO = new SwerveModuleIO[]{FL, FR, BL, BR};
        gyroIO = gyro;

        for (int i = 0; i < 4; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
        }
        for (int i = 0; i < 4; i++) {
//            this uses relative steer encoder, this could also use absolute if needed
            modulePositions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
        }

        poseEstimator = new SwerveDrivePoseEstimator(driveKinematics, new Rotation2d(), modulePositions, new Pose2d());
    }

    @Override
    public void periodic() {
        SwerveModuleState[] actualStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
            actualStates[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
        }
        Logger.getInstance().processInputs("FL Swerve Module", moduleInputs[0]);
        Logger.getInstance().processInputs("FR Swerve Module", moduleInputs[1]);
        Logger.getInstance().processInputs("BL Swerve Module", moduleInputs[2]);
        Logger.getInstance().processInputs("BR Swerve Module", moduleInputs[3]);

        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Gyro", gyroInputs);

        for (int i = 0; i < 4; i++) {
//            this uses relative steer encoder, this could also use absolute if needed
            modulePositions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
        }
        poseEstimator.update(getRotation(), modulePositions);

        Logger.getInstance().recordOutput("Pose", getPose());
        Logger.getInstance().recordOutput("Wanted States", moduleStates);
        Logger.getInstance().recordOutput("Actual States", actualStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredModuleStates) {
        SwerveModuleState[] wantedModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            wantedModuleStates[i] = SwerveOptimizer.optimize(desiredModuleStates[i], modulePositions[i].angle);
            moduleIO[i].setModuleState(SwerveOptimizer.optimize(desiredModuleStates[i], modulePositions[i].angle));
        }
        moduleStates = wantedModuleStates;
    }

    public void setModuleStates(ChassisSpeeds speeds) {
//        254 method of dealing with skew
        Pose2d robot_pose_vel = new Pose2d(speeds.vxMetersPerSecond * Constants.ROBOT_LOOP_TIME_SECONDS,
                speeds.vyMetersPerSecond * Constants.ROBOT_LOOP_TIME_SECONDS,
                Rotation2d.fromRadians(speeds.vyMetersPerSecond * Constants.ROBOT_LOOP_TIME_SECONDS));
        Twist2d twist_vel = GeomUtil.log(robot_pose_vel);
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / Constants.ROBOT_LOOP_TIME_SECONDS, twist_vel.dy / Constants.ROBOT_LOOP_TIME_SECONDS, twist_vel.dtheta / Constants.ROBOT_LOOP_TIME_SECONDS);

        SwerveModuleState[] desiredModuleStates = driveKinematics.toSwerveModuleStates(updated_chassis_speeds);
        moduleStates = desiredModuleStates;
        for (int i = 0; i < 4; i++) {
            moduleIO[i].setModuleState(SwerveOptimizer.optimize(desiredModuleStates[i], modulePositions[i].angle));
        }
    }

    public void driveFieldOrientated(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, getPose().getRotation()));
    }

//    drives wheels at x to prevent being shoved
    public void driveX() {
        setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(225)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
        });
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), modulePositions, pose);
    }

    public void addVisionData(Pose2d pose, double time) {
//        this is recommended, but I'm not sure if I like it
//        if (GeomUtil.distance(pose, getPose()) < 1) {
            poseEstimator.addVisionMeasurement(pose, time);
//        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the pigeon's angle
     * @return current angle; positive = clockwise
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromRadians(-gyroInputs.yawPositionRad);
    }
}