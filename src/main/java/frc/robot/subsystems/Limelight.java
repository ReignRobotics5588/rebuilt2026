package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Limelight 2 vision subsystem for April tag detection and robot localization.
 * Provides methods for getting robot pose, April tag information, and alignment data.
 * 
 * Network Table entries for Limelight:
 * - tx: Horizontal offset from cross-hair to target (-27 to 27 degrees)
 * - ty: Vertical offset from cross-hair to target (-20.5 to 20.5 degrees)
 * - ta: Target area (0% to 100% of image)
 * - tl: Pipeline latency in milliseconds
 * - tid: ID of detected April tag
 * - botpose: [x, y, z, roll, pitch, yaw] - robot pose in field coordinates
 * - botpose_targetspace: Robot pose relative to target
 * - camerapose_targetspace: Camera pose relative to target
 * - targetpose_cameraspace: Target pose relative to camera
 */
public class Limelight extends SubsystemBase {
  private NetworkTable m_limelightTable;
  private boolean m_hasTarget = false;
  private double m_targetOffsetX = 0.0;
  private double m_targetOffsetY = 0.0;
  private int m_targetID = -1;
  private int m_desiredTagID = -1;  // -1 means any tag

  public Limelight() {
    // Initialize network table connection to Limelight
    m_limelightTable = NetworkTableInstance.getDefault()
        .getTable(Constants.LimelightConstants.kLimelightTableName);
    
    // Set to AprilTag pipeline
    setPipeline(Constants.LimelightConstants.kAprilTagPipeline);
    
    // Initialize dashboard
    initializeDashboard();
  }

  /**
   * Initialize dashboard with target tag ID
   */
  private void initializeDashboard() {
    SmartDashboard.putNumber("Limelight/Target Tag ID", -1);
  }

  @Override
  public void periodic() {
    // Read desired tag ID from dashboard
    m_desiredTagID = (int) SmartDashboard.getNumber("Limelight/Target Tag ID", -1);
    
    // Update vision tracking data every cycle
    updateTargetData();
    updateTelemetry();
  }

  /**
   * Updates target detection data from Limelight
   */
  private void updateTargetData() {
    // Check if we have a valid target (tv = 1 if target detected)
    m_hasTarget = m_limelightTable.getEntry("tv").getDouble(0) == 1.0;
    
    if (m_hasTarget) {
      // Get horizontal and vertical offsets to target
      m_targetOffsetX = m_limelightTable.getEntry("tx").getDouble(0.0);
      m_targetOffsetY = m_limelightTable.getEntry("ty").getDouble(0.0);
      
      // Get detected April tag ID
      m_targetID = (int) m_limelightTable.getEntry("tid").getDouble(-1);
    }
  }

  /**
   * Send telemetry data to SmartDashboard/Glass for testing and review
   */
  private void updateTelemetry() {
    // Target detection status
    SmartDashboard.putBoolean("Limelight/Has Target", m_hasTarget);
    SmartDashboard.putNumber("Limelight/Target ID", m_targetID);
    SmartDashboard.putBoolean("Limelight/Has Desired Target", hasDesiredTarget());
    
    // Offsets and positioning
    SmartDashboard.putNumber("Limelight/Offset X (deg)", m_targetOffsetX);
    SmartDashboard.putNumber("Limelight/Offset Y (deg)", m_targetOffsetY);
    SmartDashboard.putNumber("Limelight/Target Area (%)", getTargetArea());
    
    // Distance measurements
    double[] poseRelative = getRobotPoseRelativeToTarget();
    SmartDashboard.putNumber("Limelight/Distance to Target (m)", poseRelative[0]);
    SmartDashboard.putNumber("Limelight/Horizontal Distance (m)", poseRelative[1]);
    
    // Field positioning
    double[] fieldPose = getRobotPose();
    SmartDashboard.putNumber("Limelight/Robot X (m)", fieldPose[0]);
    SmartDashboard.putNumber("Limelight/Robot Y (m)", fieldPose[1]);
    SmartDashboard.putNumber("Limelight/Robot Heading (deg)", fieldPose[5]);
    
    // Camera diagnostics
    SmartDashboard.putNumber("Limelight/Pipeline Latency (ms)", getPipelineLatency());
    SmartDashboard.putNumber("Limelight/Active Pipeline", getPipeline());
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
    return m_limelightTable.getEntry("ta").getDouble(0.0);
  }

  /**
   * Get pipeline latency in milliseconds
   * @return latency including capture and processing
   */
  public double getPipelineLatency() {
    return m_limelightTable.getEntry("tl").getDouble(0.0);
  }

  /**
   * Get robot pose in field coordinates from MegaTag 2
   * Returns [x, y, z, roll, pitch, yaw]
   * @return array of [x(m), y(m), z(m), roll(deg), pitch(deg), yaw(deg)], or zeros if no target
   */
  public double[] getRobotPose() {
    double[] poseArray = m_limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
    return poseArray.length == 6 ? poseArray : new double[6];
  }

  /**
   * Get robot pose relative to the current AprilTag
   * Useful for distance/angle calculations to target
   * @return array of [x(m), y(m), z(m), roll(deg), pitch(deg), yaw(deg)]
   */
  public double[] getRobotPoseRelativeToTarget() {
    double[] poseArray = m_limelightTable.getEntry("botpose_targetspace")
        .getDoubleArray(new double[6]);
    return poseArray.length == 6 ? poseArray : new double[6];
  }

  /**
   * Get camera pose relative to the current AprilTag
   * @return array of [x(m), y(m), z(m), roll(deg), pitch(deg), yaw(deg)]
   */
  public double[] getCameraPoseRelativeToTarget() {
    double[] poseArray = m_limelightTable.getEntry("camerapose_targetspace")
        .getDoubleArray(new double[6]);
    return poseArray.length == 6 ? poseArray : new double[6];
  }

  /**
   * Get distance to AprilTag in meters (X component from botpose_targetspace)
   * @return distance in meters
   */
  public double getDistanceToTarget() {
    double[] poseRelativeToTarget = getRobotPoseRelativeToTarget();
    return poseRelativeToTarget[0];  // X component is distance forward
  }

  /**
   * Get horizontal distance to AprilTag in meters (Y component)
   * @return distance in meters (positive = right, negative = left)
   */
  public double getHorizontalDistanceToTarget() {
    double[] poseRelativeToTarget = getRobotPoseRelativeToTarget();
    return poseRelativeToTarget[1];  // Y component
  }

  /**
   * Set which pipeline to use on Limelight
   * @param pipeline pipeline ID (0 = default AprilTag)
   */
  public void setPipeline(int pipeline) {
    m_limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Get current active pipeline
   * @return pipeline ID
   */
  public int getPipeline() {
    return (int) m_limelightTable.getEntry("pipeline").getDouble(0);
  }

  /**
   * Enable/disable Limelight processing
   * @param enabled true to enable LEDs and processing
   */
  public void setLimelightActive(boolean enabled) {
    // LED mode: 0 = pipeline default, 1 = force off, 2 = force blink, 3 = force on
    m_limelightTable.getEntry("ledMode").setNumber(enabled ? 0 : 1);
  }

  /**
   * Force Limelight LED on
   */
  public void ledOn() {
    m_limelightTable.getEntry("ledMode").setNumber(3);
  }

  /**
   * Force Limelight LED off
   */
  public void ledOff() {
    m_limelightTable.getEntry("ledMode").setNumber(1);
  }

  /**
   * Get debug string with current vision data
   * @return formatted string with target info
   */
  public String getDebugString() {
    if (!m_hasTarget) {
      return "[Limelight] No target detected";
    }
    
    double[] poseRelative = getRobotPoseRelativeToTarget();
    return String.format(
        "[Limelight] ID:%d | Offset:(%.1f°, %.1f°) | Dist:%.2fm | Area:%.1f%%",
        m_targetID, 
        m_targetOffsetX, 
        m_targetOffsetY, 
        poseRelative[0],
        getTargetArea()
    );
  }
}
