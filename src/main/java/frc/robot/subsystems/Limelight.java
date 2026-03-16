package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

/**
 * Limelight 2 vision subsystem for April tag detection and robot localization.
 * Uses the official LimelightHelpers library for reliable vision integration.
 * Provides methods for getting robot pose, April tag information, and alignment data.
 */
public class Limelight extends SubsystemBase {
  private final String m_limelightName;
  private boolean m_hasTarget = false;
  private double m_targetOffsetX = 0.0;
  private double m_targetOffsetY = 0.0;
  private int m_targetID = -1;
  private int m_desiredTagID = -1;  // -1 means any tag
  private PoseEstimate m_latestPoseEstimate = new PoseEstimate();

  public Limelight() {
    m_limelightName = Constants.LimelightConstants.kLimelightTableName;
    
    // Set to AprilTag pipeline
    LimelightHelpers.setPipelineIndex(m_limelightName, Constants.LimelightConstants.kAprilTagPipeline);

    LimelightHelpers.setLEDMode_ForceOff(m_limelightName);
    
    // Initialize dashboard
    initializeDashboard();
  }

  /**
   * Initialize dashboard with target tag ID
   */
  private void initializeDashboard() {
    // Initialize desired tag ID key (-1 = any tag)
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardTargetTagIdKey, -1);
    // Only publish the minimal keys needed for tuning angling and distance
    SmartDashboard.putBoolean(Constants.LimelightConstants.kDashboardHasTargetKey, false);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetXKey, 0.0);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetYKey, 0.0);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardDistanceKey, 0.0);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardHorizontalDistanceKey, 0.0);
  }

  @Override
  public void periodic() {
    // Update vision tracking data every cycle
    updateTargetData();
    updateTelemetry();
  }

  /**
   * Updates target detection data from Limelight using the official LimelightHelpers
   */
  private void updateTargetData() {
    // Get the latest pose estimate based on alliance color
    String allianceName = DriverStation.getAlliance().map(Enum::name).orElse("Invalid");
    PoseEstimate poseEst;
    
    if ("RED".equalsIgnoreCase(allianceName)) {
      poseEst = LimelightHelpers.getBotPoseEstimate_wpiRed(m_limelightName);
    } else if ("BLUE".equalsIgnoreCase(allianceName)) {
      poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);
    } else {
      // Fall back to generic pose estimate
      poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);
    }
    
    m_latestPoseEstimate = poseEst;
    
    // Check if we have valid targets using official helpers
    m_hasTarget = LimelightHelpers.getTV(m_limelightName);
    
    if (m_hasTarget && poseEst.rawFiducials != null && poseEst.rawFiducials.length > 0) {
      // Use the first (best) fiducial detected
      RawFiducial primaryFiducial = poseEst.rawFiducials[0];
      m_targetID = primaryFiducial.id;
      m_targetOffsetX = primaryFiducial.txnc;
      m_targetOffsetY = primaryFiducial.tync;
      
      // If desired tag is set, check if this is the one we want
      if (m_desiredTagID != -1 && m_targetID != m_desiredTagID) {
        // Look for the desired tag among all detected fiducials
        for (RawFiducial fid : poseEst.rawFiducials) {
          if (fid.id == m_desiredTagID) {
            m_targetID = (int) fid.id;
            m_targetOffsetX = fid.txnc;
            m_targetOffsetY = fid.tync;
            break;
          }
        }
      }
    } else {
      m_hasTarget = false;
      m_targetID = -1;
      m_targetOffsetX = 0.0;
      m_targetOffsetY = 0.0;
    }
  }

  /**
   * Send telemetry data to SmartDashboard/Glass for testing and review
   */
  private void updateTelemetry() {
    // Target detection status
    // Only publish data required for tuning angling and distance
    SmartDashboard.putBoolean(Constants.LimelightConstants.kDashboardHasTargetKey, m_hasTarget);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetXKey, m_targetOffsetX);
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardOffsetYKey, m_targetOffsetY);
    double[] poseRelative = getRobotPoseRelativeToTarget();
    if (poseRelative != null && poseRelative.length >= 2) {
      SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardDistanceKey, poseRelative[0]);
      SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardHorizontalDistanceKey, poseRelative[1]);
    } else {
      SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardDistanceKey, 0.0);
      SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardHorizontalDistanceKey, 0.0);
    }
  }

  /**
   * Check if Limelight detects a valid target
   * @return true if April tag is detected
   */
  public boolean hasValidTarget() {
    return m_hasTarget;
  }

  /**
   * Check if the desired April tag ID is currently detected.
   * If desired tag ID is -1, any tag is considered valid.
   * @return true if the correct tag is detected or any tag if ID is -1
   */
  public boolean hasDesiredTarget() {
    if (!m_hasTarget) {
      return false;
    }
    // If desired tag ID is -1, accept any tag
    if (m_desiredTagID == -1) {
      return true;
    }
    // Otherwise check if detected tag matches desired ID
    return m_targetID == m_desiredTagID;
  }

  /**
   * Get the desired April tag ID from the dashboard (-1 means any tag)
   * @return desired tag ID or -1 for any tag
   */
  public int getDesiredTagID() {
    return m_desiredTagID;
  }

  /**
   * Set the desired April tag ID to target
   * @param tagID the tag ID to target, or -1 for any tag
   */
  public void setDesiredTagID(int tagID) {
    m_desiredTagID = tagID;
    SmartDashboard.putNumber(Constants.LimelightConstants.kDashboardTargetTagIdKey, m_desiredTagID);
  }

  /**
   * Get horizontal offset to target in degrees
   * Positive = target is to the right
   * Negative = target is to the left
   * @return offset in degrees (-27 to 27)
   */
  public double getTargetOffsetX() {
    return m_targetOffsetX;
  }

  /**
   * Get vertical offset to target in degrees
   * Positive = target is above cross-hair
   * Negative = target is below cross-hair
   * @return offset in degrees (-20.5 to 20.5)
   */
  public double getTargetOffsetY() {
    return m_targetOffsetY;
  }

  /**
   * Get the ID of the currently detected April tag
   * @return April tag ID, or -1 if no tag detected
   */
  public int getTargetID() {
    return m_targetID;
  }

  /**
   * Get target area as percentage of image
   * @return area percentage (0-100)
   */
  public double getTargetArea() {
    return LimelightHelpers.getTA(m_limelightName);
  }

  /**
   * Get pipeline latency in milliseconds
   * @return latency including capture and processing
   */
  public double getPipelineLatency() {
    return LimelightHelpers.getLatency_Pipeline(m_limelightName);
  }

  /**
   * Get robot pose in field coordinates from MegaTag 2
   * Returns [x, y, z, roll, pitch, yaw]
   * @return array of [x(m), y(m), z(m), roll(deg), pitch(deg), yaw(deg)], or zeros if no target
   */
  public double[] getRobotPose() {
    return LimelightHelpers.getBotPose(m_limelightName);
  }

  /**
   * Get robot pose relative to the current AprilTag
   * Useful for distance/angle calculations to target
   * @return array of [x(m), y(m), z(m), roll(deg), pitch(deg), yaw(deg)]
   */
  public double[] getRobotPoseRelativeToTarget() {
    return LimelightHelpers.getBotPose_TargetSpace(m_limelightName);
  }

  /**
   * Get camera pose relative to the current AprilTag
   * @return array of [x(m), y(m), z(m), roll(deg), pitch(deg), yaw(deg)]
   */
  public double[] getCameraPoseRelativeToTarget() {
    return LimelightHelpers.getCameraPose_TargetSpace(m_limelightName);
  }

  /**
   * Get distance to AprilTag in meters (X component from botpose_targetspace)
   * @return distance in meters
   */
  public double getDistanceToTarget() {
    double[] poseRelativeToTarget = getRobotPoseRelativeToTarget();
    if (poseRelativeToTarget != null && poseRelativeToTarget.length > 0) {
      return poseRelativeToTarget[0];  // X component is distance forward
    }
    return 0.0;
  }

  /**
   * Get horizontal distance to AprilTag in meters (Y component)
   * @return distance in meters (positive = right, negative = left)
   */
  public double getHorizontalDistanceToTarget() {
    double[] poseRelativeToTarget = getRobotPoseRelativeToTarget();
    if (poseRelativeToTarget != null && poseRelativeToTarget.length > 1) {
      return poseRelativeToTarget[1];  // Y component
    }
    return 0.0;
  }

  /**
   * Set which pipeline to use on Limelight
   * @param pipeline pipeline ID (0 = default AprilTag)
   */
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(m_limelightName, pipeline);
  }

  /**
   * Get current active pipeline
   * @return pipeline ID
   */
  public int getPipeline() {
    return (int) LimelightHelpers.getCurrentPipelineIndex(m_limelightName);
  }

  /**
   * Enable/disable Limelight processing
   * @param enabled true to enable LEDs and processing
   */
  public void setLimelightActive(boolean enabled) {
    // LED mode: 0 = pipeline default, 1 = force off, 2 = force blink, 3 = force on
    if (enabled) {
      LimelightHelpers.setLEDMode_PipelineControl(m_limelightName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(m_limelightName);
    }
  }

  /**
   * Force Limelight LED on
   */
  public void ledOn() {
    LimelightHelpers.setLEDMode_ForceOn(m_limelightName);
  }

  /**
   * Force Limelight LED off
   */
  public void ledOff() {
    LimelightHelpers.setLEDMode_ForceOff(m_limelightName);
  }

  /**
   * Get the latest pose estimate from the Limelight including timestamp and tag count
   * Useful for integrating with robot odometry and pose estimation
   * @return PoseEstimate object with pose, latency, timestamp, and raw fiducials
   */
  public PoseEstimate getLatestPoseEstimate() {
    return m_latestPoseEstimate;
  }
  public String getDebugString() {
    if (!m_hasTarget) {
      return "[Limelight] No target detected";
    }
    
    double[] poseRelative = getRobotPoseRelativeToTarget();
    double distance = (poseRelative != null && poseRelative.length > 0) ? poseRelative[0] : 0.0;
    return String.format(
        "[Limelight] ID:%d | Offset:(%.1f°, %.1f°) | Dist:%.2fm | Area:%.1f%%",
        m_targetID, 
        m_targetOffsetX, 
        m_targetOffsetY, 
        distance,
        getTargetArea()
    );
  }
}
