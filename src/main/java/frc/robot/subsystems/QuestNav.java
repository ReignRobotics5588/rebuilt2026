package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * QuestNav vision subsystem for full field robot localization and pose estimation.
 * Uses Meta Quest headset with visual inertial odometry (VIO) and AprilTag fusion
 * to provide sub-centimeter accurate robot pose tracking across the entire field.
 * 
 * Provides methods for:
 * - Getting robot pose in field coordinates
 * - Tracking robot movement and rotation
 * - Integrating with robot odometry for improved pose estimation
 */
public class QuestNav extends SubsystemBase {
  // Reference to the QuestNav library instance
  private final gg.questnav.questnav.QuestNav m_questNav = new gg.questnav.questnav.QuestNav();
  
  // Mount offset: transform from Quest coordinate frame to robot coordinate frame
  // TODO: Calibrate this transform based on your physical Quest mounting orientation
  private static final Transform3d ROBOT_TO_QUEST = new Transform3d();
  
  private Pose2d m_robotPose = new Pose2d();
  private boolean m_hasValidPose = false;
  private boolean m_isTracking = false;
  private double m_lastUpdateTime = 0;
  
  public QuestNav() {
    // Initialize QuestNav connection
    // QuestNav runs on the Quest headset connected to the RoboRIO
    initializeDashboard();
  }

  /**
   * Initialize dashboard with QuestNav telemetry
   */
  private void initializeDashboard() {
    SmartDashboard.putNumber("QuestNav/Robot X (m)", 0.0);
    SmartDashboard.putNumber("QuestNav/Robot Y (m)", 0.0);
    SmartDashboard.putNumber("QuestNav/Robot Heading (deg)", 0.0);
    SmartDashboard.putBoolean("QuestNav/Has Valid Pose", false);
    SmartDashboard.putBoolean("QuestNav/Is Tracking", false);
    SmartDashboard.putString("QuestNav/Status", "Initializing...");
    SmartDashboard.putNumber("QuestNav/Update Latency (ms)", 0.0);
  }

  @Override
  public void periodic() {
    // Must call commandPeriodic() to allow QuestNav to process new data
    m_questNav.commandPeriodic();
    
    // Update pose from QuestNav
    updatePoseData();
    updateTelemetry();
  }

  /**
   * Updates robot pose data from QuestNav
   * QuestNav provides pose frames via getAllUnreadPoseFrames()
   */
  private void updatePoseData() {
    try {
      // Get all unread pose frames since last call
      Object[] frames = (Object[]) m_questNav.getAllUnreadPoseFrames();
      
      if (frames != null && frames.length > 0) {
        // Use the most recent frame
        Object latestFrame = frames[frames.length - 1];
        
        // Use reflection to safely call methods on PoseFrame
        // Check if Quest is currently tracking
        m_isTracking = (boolean) latestFrame.getClass().getMethod("isTracking").invoke(latestFrame);
        
        if (m_isTracking) {
          // Get the 3D pose from Quest
          Pose3d questPose3d = (Pose3d) latestFrame.getClass().getMethod("questPose3d").invoke(latestFrame);
          
          // Transform from Quest frame to robot frame using mount offset
          Pose3d robotPose3d = questPose3d.transformBy(ROBOT_TO_QUEST.inverse());
          
          // Convert 3D pose to 2D (project onto XY plane, use rotation around Z)
          m_robotPose = new Pose2d(
              robotPose3d.getX(),
              robotPose3d.getY(),
              robotPose3d.getRotation().toRotation2d()
          );
          
          m_hasValidPose = true;
          m_lastUpdateTime = System.currentTimeMillis();
        } else {
          m_hasValidPose = false;
        }
      } else {
        m_hasValidPose = false;
        m_isTracking = false;
      }
    } catch (Exception e) {
      m_hasValidPose = false;
      m_isTracking = false;
      SmartDashboard.putString("QuestNav/Status", "Error: " + e.getMessage());
    }
  }

  /**
   * Send telemetry data to SmartDashboard for monitoring
   */
  private void updateTelemetry() {
    SmartDashboard.putNumber("QuestNav/Robot X (m)", m_robotPose.getX());
    SmartDashboard.putNumber("QuestNav/Robot Y (m)", m_robotPose.getY());
    SmartDashboard.putNumber("QuestNav/Robot Heading (deg)", m_robotPose.getRotation().getDegrees());
    SmartDashboard.putBoolean("QuestNav/Has Valid Pose", m_hasValidPose);
    SmartDashboard.putBoolean("QuestNav/Is Tracking", m_isTracking);
    
    if (m_hasValidPose && m_isTracking) {
      SmartDashboard.putString("QuestNav/Status", "Valid Pose - Tracking");
      SmartDashboard.putNumber("QuestNav/Update Latency (ms)", System.currentTimeMillis() - m_lastUpdateTime);
    } else if (m_hasValidPose) {
      SmartDashboard.putString("QuestNav/Status", "Valid Pose - Not Tracking");
    } else {
      SmartDashboard.putString("QuestNav/Status", "No Valid Pose");
    }
  }

  /**
   * Get the current robot pose in field coordinates
   * @return Pose2d containing position (x, y) and rotation
   */
  public Pose2d getRobotPose() {
    return m_robotPose;
  }

  /**
   * Get the robot's X position on the field in meters
   * @return X position (0 = near wall, increases toward far wall)
   */
  public double getRobotX() {
    return m_robotPose.getX();
  }

  /**
   * Get the robot's Y position on the field in meters
   * @return Y position
   */
  public double getRobotY() {
    return m_robotPose.getY();
  }

  /**
   * Get the robot's heading/rotation angle in degrees
   * @return rotation in degrees (0 = facing right, 90 = facing away from wall, etc)
   */
  public double getRobotHeading() {
    return m_robotPose.getRotation().getDegrees();
  }

  /**
   * Get the robot's rotation as a Rotation2d object
   * @return Rotation2d object
   */
  public Rotation2d getRobotRotation() {
    return m_robotPose.getRotation();
  }

  /**
   * Check if QuestNav has a valid pose estimate
   * @return true if pose data is valid and recent
   */
  public boolean hasValidPose() {
    return m_hasValidPose;
  }

  /**
   * Get the confidence metric of the current pose estimate
   * @return update latency in milliseconds
   */
  public double getPoseConfidence() {
    return m_hasValidPose ? 1.0 : 0.0;
  }

  /**
   * Get the latency of the latest pose update
   * @return latency in milliseconds
   */
  public double getUpdateLatency() {
    return System.currentTimeMillis() - m_lastUpdateTime;
  }

  /**
   * Reset the robot's pose to a known value
   * Useful for initialization or when you know the absolute position
   * @param newPose the new pose to set
   */
  public void resetPose(Pose2d newPose) {
    m_robotPose = newPose;
    SmartDashboard.putString("QuestNav/Status", "Pose Reset");
  }

  /**
   * Reset the robot pose to origin (0, 0, 0 degrees)
   */
  public void resetPoseToOrigin() {
    resetPose(new Pose2d());
  }

  /**
   * Get distance to a target pose in meters
   * @param targetPose the target position
   * @return distance in meters
   */
  public double getDistanceTo(Pose2d targetPose) {
    return m_robotPose.getTranslation().getDistance(targetPose.getTranslation());
  }

  /**
   * Get the angle to a target pose from current heading
   * @param targetPose the target position
   * @return angle error in degrees (positive = target is counterclockwise, negative = clockwise)
   */
  public double getAngleTo(Pose2d targetPose) {
    var toTarget = targetPose.getTranslation().minus(m_robotPose.getTranslation());
    var angleToTarget = Math.atan2(toTarget.getY(), toTarget.getX());
    return Math.toDegrees(angleToTarget) - m_robotPose.getRotation().getDegrees();
  }


  /**
   * Get a debug string with current pose information
   * @return formatted string with pose data
   */
  public String getDebugString() {
    if (!m_hasValidPose) {
      return "[QuestNav] No valid pose available";
    }
    
    return String.format(
        "[QuestNav] Pos:(%.2f, %.2f) Heading:%.1f° Confidence:%.2f Latency:%.0fms",
        m_robotPose.getX(),
        m_robotPose.getY(),
        m_robotPose.getRotation().getDegrees(),
        getPoseConfidence(),
        getUpdateLatency()
    );
  }
}
