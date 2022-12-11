package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import frc.robot.Constants;

public class GyroIOPigeon implements GyroIO{
    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(Constants.DRIVE_CAN_PIGEON);

    public GyroIOPigeon() {
        pigeon.configAllSettings(Constants.DRIVE_PIGEON_CONFIG);
        pigeon.calibrate();
        pigeon.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getUpTime() > 0;
        inputs.yawPositionRad = Math.toRadians(pigeon.getAngle());
        inputs.yawVelocityRadPerSec = Math.toRadians(pigeon.getRate());
        inputs.pitchPositionRad = Math.toRadians(pigeon.getRoll());
        inputs.rollPositionRad = Math.toRadians(pigeon.getPitch());
    }
}