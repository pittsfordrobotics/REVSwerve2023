package frc.robot.subsystems.swerve;

public class GyroIOSim implements GyroIO{

    public GyroIOSim() {
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPositionRad = 0;
        inputs.yawVelocityRadPerSec = 0;
        inputs.pitchPositionRad = 0;
        inputs.rollPositionRad = 0;
    }
}