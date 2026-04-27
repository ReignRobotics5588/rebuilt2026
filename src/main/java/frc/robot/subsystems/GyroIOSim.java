package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
  private Rotation2d yawPosition = new Rotation2d();
  private double yawVelocityRadPerSec = 0.0;

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = yawPosition;
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
  }

  /** Called by DriveSubsystem each cycle to push simulated chassis rotation into the gyro. */
  public void updateSimState(Rotation2d yaw, double yawVelocityRadPerSec) {
    this.yawPosition = yaw;
    this.yawVelocityRadPerSec = yawVelocityRadPerSec;
  }
}
