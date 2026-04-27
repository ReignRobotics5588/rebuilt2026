package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelVelocityRPM = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelCurrentAmps = 0.0;
    public double indexerAppliedVolts = 0.0;
    public double indexerCurrentAmps = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set flywheel to a target RPM using closed-loop control. */
  public default void setFlywheelRPM(double rpm) {}

  /** Set flywheel to a raw percent speed (-1 to 1). */
  public default void setFlywheelSpeed(double speed) {}

  /** Set indexer to a raw percent speed (-1 to 1). */
  public default void setIndexerSpeed(double speed) {}

  /** Update internal PID gains (used for real-hardware tuning from dashboard). */
  public default void updateFlywheelPID(double p, double i, double d) {}
}
