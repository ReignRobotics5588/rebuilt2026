package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public boolean hasTarget = false;
    public double targetOffsetX = 0.0;
    public double targetOffsetY = 0.0;
    public int targetID = -1;
    public double targetArea = 0.0;
    public double distanceToTarget = 0.0;
    public double horizontalDistanceToTarget = 0.0;
    public double[] poseRelativeToTarget = new double[6];
    public double pipelineLatencyMs = 0.0;
    public int activePipeline = 0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setDesiredTagID(int tagID) {}

  public default void setPipeline(int pipeline) {}

  public default void setLEDs(boolean on) {}
}
