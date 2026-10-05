package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

import java.util.Optional;

public class Limelight extends SubsystemBase {

  private final limelight.Limelight m_limelight;
  private final LimelightPoseEstimator m_poseEstimator;

  private boolean m_hasTarget = false;
  private double m_targetOffsetX = 0.0;
  private double m_targetOffsetY = 0.0;
  private int m_targetID = -1;
  private int m_desiredTagID = -1;
  private double m_targetArea = 0.0;
  private double m_distToCamera = 0.0;
  private Optional<PoseEstimate> m_latestPoseEstimate = Optional.empty();

  public Limelight() {
    m_limelight = new limelight.Limelight(Constants.LimelightConstants.kLimelightTableName);
    m_poseEstimator = m_limelight.createPoseEstimator(EstimationMode.MEGATAG2);

    m_limelight.getSettings()
        .withPipelineIndex(Constants.LimelightConstants.kAprilTagPipeline)
        .withLimelightLEDMode(LEDMode.ForceOff);

    initializeDashboard();
  }

  private void initializeDashboard() {
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardTargetTagIdKey, -1);
    SmartDashboard.putBoolean(Constants.LimelightConstants.kDashboardHasTargetKey, false);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetXKey, 0.0);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetYKey, 0.0);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardDistanceKey, 0.0);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardHorizontalDistanceKey, 0.0);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardTargetAreaKey, 0.0);
  }

  @Override
  public void periodic() {
    updateTargetData();
    updateTelemetry();
  }

  private void updateTargetData() {
    m_latestPoseEstimate = m_poseEstimator.getAlliancePoseEstimate();

    var targetData = m_limelight.getData().targetData;
    m_hasTarget = targetData.getTargetStatus();

    if (m_hasTarget) {
      m_targetOffsetX = targetData.getHorizontalOffset();
      m_targetOffsetY = targetData.getVerticalOffset();
      m_targetArea = targetData.getTargetArea();
      m_targetID = (int) targetData.getAprilTagID();

      RawFiducial[] fiducials = m_limelight.getData().getRawFiducials();

      if (m_desiredTagID != -1 && m_targetID != m_desiredTagID) {
        for (RawFiducial fid : fiducials) {
          if (fid.id == m_desiredTagID) {
            m_targetID = fid.id;
            m_targetOffsetX = fid.txnc;
            m_targetOffsetY = fid.tync;
            m_targetArea = fid.ta;
            m_distToCamera = fid.distToCamera;
            break;
          }
        }
      } else if (fiducials.length > 0) {
        m_distToCamera = fiducials[0].distToCamera;
      }
    } else {
      m_targetID = -1;
      m_targetOffsetX = 0.0;
      m_targetOffsetY = 0.0;
      m_targetArea = 0.0;
      m_distToCamera = 0.0;
    }
  }

  private void updateTelemetry() {
    SmartDashboard.putBoolean(Constants.LimelightConstants.kDashboardHasTargetKey, m_hasTarget);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetXKey, m_targetOffsetX);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetYKey, m_targetOffsetY);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardTargetAreaKey, m_targetArea);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardDistanceKey, m_distToCamera);

    var robotToTarget = m_limelight.getData().targetData.getRobotToTarget();
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardDistanceKey, robotToTarget.getX());
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardHorizontalDistanceKey, robotToTarget.getY());
  }

  public boolean hasValidTarget() {
    return m_hasTarget;
  }

  public boolean hasDesiredTarget() {
    if (!m_hasTarget) return false;
    if (m_desiredTagID == -1) return true;
    return m_targetID == m_desiredTagID;
  }

  public int getDesiredTagID() {
    return m_desiredTagID;
  }

  public void setDesiredTagID(int tagID) {
    m_desiredTagID = tagID;
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardTargetTagIdKey, tagID);
    if (tagID != -1) {
      m_limelight.getSettings().withPriorityTagId(tagID);
    }
  }

  public double getTargetOffsetX() {
    return m_targetOffsetX;
  }

  public double getTargetOffsetY() {
    return m_targetOffsetY;
  }

  public int getTargetID() {
    return m_targetID;
  }

  public double getTargetArea() {
    return m_limelight.getData().targetData.getTargetArea();
  }

  public double getPipelineLatency() {
    double[] metrics = m_limelight.getData().targetData.getTargetMetrics();
    // t2d array: [targetValid, targetCount, targetLatency, captureLatency, ...]
    if (metrics.length >= 3) {
      return metrics[2];
    }
    return 0.0;
  }

  public double[] getRobotPose() {
    return m_latestPoseEstimate.map(pe -> {
      var p = pe.pose;
      var rot = p.getRotation();
      return new double[]{
          p.getX(), p.getY(), p.getZ(),
          Math.toDegrees(rot.getX()), Math.toDegrees(rot.getY()), Math.toDegrees(rot.getZ())
      };
    }).orElse(new double[6]);
  }

  public double[] getRobotPoseRelativeToTarget() {
    var pose = m_limelight.getData().targetData.getRobotToTarget();
    var rot = pose.getRotation();
    return new double[]{
        pose.getX(), pose.getY(), pose.getZ(),
        Math.toDegrees(rot.getX()), Math.toDegrees(rot.getY()), Math.toDegrees(rot.getZ())
    };
  }

  public double[] getCameraPoseRelativeToTarget() {
    var pose = m_limelight.getData().targetData.getCameraToTarget();
    var rot = pose.getRotation();
    return new double[]{
        pose.getX(), pose.getY(), pose.getZ(),
        Math.toDegrees(rot.getX()), Math.toDegrees(rot.getY()), Math.toDegrees(rot.getZ())
    };
  }

  public double getDistanceToTarget() {
    return m_limelight.getData().targetData.getRobotToTarget().getX();
  }

  public double getHorizontalDistanceToTarget() {
    return m_limelight.getData().targetData.getRobotToTarget().getY();
  }

  public void setPipeline(int pipeline) {
    m_limelight.getSettings().withPipelineIndex(pipeline);
  }

  public int getPipeline() {
    return Constants.LimelightConstants.kAprilTagPipeline;
  }

  public void setLimelightActive(boolean enabled) {
    m_limelight.getSettings().withLimelightLEDMode(
        enabled ? LEDMode.PipelineControl : LEDMode.ForceOff);
  }

  public void ledOn() {
    m_limelight.getSettings().withLimelightLEDMode(LEDMode.ForceOn);
  }

  public void ledOff() {
    m_limelight.getSettings().withLimelightLEDMode(LEDMode.ForceOff);
  }

  public Optional<PoseEstimate> getLatestPoseEstimate() {
    return m_latestPoseEstimate;
  }

  public String getDebugString() {
    if (!m_hasTarget) {
      return "[Limelight] No target detected";
    }
    return String.format(
        "[Limelight] ID:%d | Offset:(%.1f°, %.1f°) | Dist:%.2fm | Area:%.1f%%",
        m_targetID, m_targetOffsetX, m_targetOffsetY,
        getDistanceToTarget(), getTargetArea());
  }
}
