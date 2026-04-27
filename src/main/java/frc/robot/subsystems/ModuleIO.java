package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public boolean turnConnected = false;
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Set the drive motor to a target velocity in meters per second. */
  public default void setDriveVelocity(double velocityMetersPerSec) {}

  /** Set the turn motor to a target angle in radians (robot-relative, offset already removed). */
  public default void setTurnPosition(double positionRadians) {}

  public default void resetEncoders() {}
}
