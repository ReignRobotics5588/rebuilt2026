package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 gyro = new Pigeon2(DriveConstants.kGyroID);

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble() % 360);
    inputs.yawVelocityRadPerSec = Math.toRadians(
        gyro.getAngularVelocityZDevice().getValueAsDouble()
            * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
  }

  @Override
  public void reset() {
    gyro.reset();
  }
}
