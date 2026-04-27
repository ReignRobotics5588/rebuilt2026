package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface BeltIO {

  @AutoLog
  public static class BeltIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocityRPM = 0.0;
  }

  public default void updateInputs(BeltIOInputs inputs) {}

  public default void setSpeed(double speed) {}
}
