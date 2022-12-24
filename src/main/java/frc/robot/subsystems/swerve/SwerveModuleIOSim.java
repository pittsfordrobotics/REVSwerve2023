package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private FlywheelSim driveSim =
            new FlywheelSim(DCMotor.getNEO(1), Constants.SWERVE_DRIVE_GEAR_RATIO, 0.025);
    private FlywheelSim turnSim =
            new FlywheelSim(DCMotor.getNEO(1), Constants.SWERVE_STEER_GEAR_RATIO, 0.004096955);
    private PIDController drivePID = new PIDController(0.6, 0, 0);
    private PIDController steerPID = new PIDController(0.1, 0, 0);

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveSim.update(Constants.ROBOT_LOOP_TIME_SECONDS);
        turnSim.update(Constants.ROBOT_LOOP_TIME_SECONDS);

        double angleDiffRad =
                turnSim.getAngularVelocityRadPerSec() * Constants.ROBOT_LOOP_TIME_SECONDS;
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * Math.PI * Constants.SWERVE_WHEEL_DIAMETER_METERS / 6.75;
        inputs.drivePositionMeters = inputs.drivePositionMeters
                + (inputs.driveVelocityMetersPerSec * Constants.ROBOT_LOOP_TIME_SECONDS);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.driveTempCelcius = 0;

        inputs.steerAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.steerAbsoluteVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.steerPositionRad = turnRelativePositionRad;
        inputs.steerVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = turnAppliedVolts;
        inputs.steerCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        inputs.steerTempCelcius = 0;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setSteerVoltage(double voltage) {
        turnAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    @Override
    public void setModuleState(SwerveModuleState state) {
        driveSim.setInputVoltage(drivePID.calculate(state.speedMetersPerSecond));
        turnSim.setInputVoltage(drivePID.calculate(turnAbsolutePositionRad - state.angle.getRadians()));
    }
}