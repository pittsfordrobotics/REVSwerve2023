package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.LazySparkMax;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final LazySparkMax driveMotor;
    private final LazySparkMax steerMotor;

    private final RelativeEncoder driveRelativeEncoder;
    private final RelativeEncoder steerRelativeEncoder;
    private final RelativeEncoder steerAbsoluteEncoder;

    public SwerveModuleIOSparkMax(int driveID, int steerID, Rotation2d offset) {
        driveMotor = new LazySparkMax(driveID, IdleMode.kBrake, 80);
        steerMotor = new LazySparkMax(steerID, IdleMode.kBrake, 30);

        driveRelativeEncoder = driveMotor.getEncoder();
        steerRelativeEncoder = steerMotor.getEncoder();
        steerAbsoluteEncoder = steerMotor.getAlternateEncoder(Constants.SWERVE_THROUGH_BORE_COUNTS_PER_REV);

        driveRelativeEncoder.setPositionConversionFactor(Math.PI * Constants.SWERVE_WHEEL_DIAMETER / Constants.SWERVE_DRIVE_GEAR_RATIO);
        driveRelativeEncoder.setVelocityConversionFactor(Math.PI * Constants.SWERVE_WHEEL_DIAMETER / Constants.SWERVE_DRIVE_GEAR_RATIO / 60);
        steerRelativeEncoder.setPositionConversionFactor(2 * Math.PI / Constants.SWERVE_STEER_GEAR_RATIO);
        steerRelativeEncoder.setVelocityConversionFactor(2 * Math.PI / Constants.SWERVE_STEER_GEAR_RATIO / 60);
        steerAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        steerAbsoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60);

        steerRelativeEncoder.setPosition(steerAbsoluteEncoder.getPosition() - offset.getRadians());
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveRelativeEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = driveRelativeEncoder.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelcius = driveMotor.getMotorTemperature();

        inputs.steerAbsolutePositionRad = steerAbsoluteEncoder.getPosition();
        inputs.steerAbsoluteVelocityRadPerSec = steerAbsoluteEncoder.getVelocity();
        inputs.steerPositionRad = steerRelativeEncoder.getPosition();
        inputs.steerVelocityRadPerSec = steerRelativeEncoder.getVelocity();
        inputs.steerAppliedVolts = steerMotor.getAppliedVoltage();
        inputs.steerCurrentAmps = steerMotor.getOutputCurrent();
        inputs.steerTempCelcius = steerMotor.getMotorTemperature();
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    @Override
    public void setSteerVoltage(double voltage) {
        steerMotor.setVoltage(voltage);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setSteerBrakeMode(boolean enable) {
        steerMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}