package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  private int m_desiredTagID = -1;

  public Limelight(VisionIO io) {
    this.io = io;
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardTargetTagIdKey, -1);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    // Keep SmartDashboard values in sync for backwards-compatible dashboard displays
    SmartDashboard.putBoolean(Constants.LimelightConstants.kDashboardHasTargetKey, inputs.hasTarget);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetXKey, inputs.targetOffsetX);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetYKey, inputs.targetOffsetY);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardTargetAreaKey, inputs.targetArea);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardDistanceKey, inputs.distanceToTarget);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardHorizontalDistanceKey,
        inputs.horizontalDistanceToTarget);
  }

  public boolean hasValidTarget() {
    return inputs.hasTarget;
  }

  public boolean hasDesiredTarget() {
    if (!inputs.hasTarget) return false;
    return m_desiredTagID == -1 || inputs.targetID == m_desiredTagID;
  }

  public int getDesiredTagID() {
    return m_desiredTagID;
  }

  public void setDesiredTagID(int tagID) {
    m_desiredTagID = tagID;
    io.setDesiredTagID(tagID);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardTargetTagIdKey, tagID);
  }

  public double getTargetOffsetX() {
    return inputs.targetOffsetX;
  }

  public double getTargetOffsetY() {
    return inputs.targetOffsetY;
  }

  public int getTargetID() {
    return inputs.targetID;
  }

  public double getTargetArea() {
    return inputs.targetArea;
  }

  public double getPipelineLatency() {
    return inputs.pipelineLatencyMs;
  }

  public double[] getRobotPoseRelativeToTarget() {
    return inputs.poseRelativeToTarget;
  }

  /** Returns a stub PoseEstimate — use getRobotPoseRelativeToTarget() for pose data in sim. */
  public PoseEstimate getLatestPoseEstimate() {
    return new PoseEstimate();
  }

  public double getDistanceToTarget() {
    return inputs.distanceToTarget;
  }

  public double getHorizontalDistanceToTarget() {
    return inputs.horizontalDistanceToTarget;
  }

  public void setPipeline(int pipeline) {
    io.setPipeline(pipeline);
  }

  public int getPipeline() {
    return inputs.activePipeline;
  }

  public void setLimelightActive(boolean enabled) {
    io.setLEDs(enabled);
  }

  public void ledOn() {
    io.setLEDs(true);
  }

  public void ledOff() {
    io.setLEDs(false);
  }

  public double[] getRobotPose() {
    double[] pose = inputs.poseRelativeToTarget;
    return (pose != null) ? pose : new double[6];
  }

  public double[] getCameraPoseRelativeToTarget() {
    // Camera pose is not tracked through the IO inputs; return zeros in sim
    return new double[6];
  }

  public String getDebugString() {
    if (!inputs.hasTarget) return "[Vision] No target";
    return String.format("[Vision] ID:%d | Offset:(%.1f°, %.1f°) | Dist:%.2fm | Area:%.1f%%",
        inputs.targetID, inputs.targetOffsetX, inputs.targetOffsetY,
        inputs.distanceToTarget, inputs.targetArea);
  }
}
