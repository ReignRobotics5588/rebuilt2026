package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface DriveIO {
  @AutoLog
  class DriveIOInputs {
    public double[] pose = new double[] {0.0, 0.0, 0.0}; // x, y, rotationDeg
    public double chassisVxMetersPerSec = 0.0;
    public double chassisVyMetersPerSec = 0.0;
    public double chassisOmegaRadPerSec = 0.0;
    public double gyroYawDegrees = 0.0;
  }

  default void init(SubsystemBase subsystem) {}

  default void updateInputs(DriveIOInputs inputs) {}

  default void drive(double xMetersPerSec, double yMetersPerSec, double rotRadPerSec,
      boolean fieldRelative) {}

  default void setX() {}

  default void resetOdometry(Pose2d pose) {}

  default void addVisionMeasurement(Pose2d visionPose, double timestampSeconds,
      Matrix<N3, N1> stdDevs) {}

  default void zeroHeading() {}

  default void simIterate() {}
}
