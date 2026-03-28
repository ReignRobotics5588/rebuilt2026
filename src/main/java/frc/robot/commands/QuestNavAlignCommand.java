package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.QuestNav;

/**
 * Command to navigate the robot to a target pose using QuestNav full-field pose tracking.
 * 
 * Uses PID controllers to:
 * - Drive toward the target position
 * - Rotate to face the target heading
 * 
 * This command provides full autonomous navigation without needing vision targets,
 * using only the robot's current pose from QuestNav and the desired target pose.
 * 
 * The command completes when the robot is within tolerance of the target pose.
 */
public class QuestNavAlignCommand extends Command {
  private final DriveSubsystem m_drive;
  private final QuestNav m_questNav;
  private final Pose2d m_targetPose;
  private final PIDController m_headingController;
  private final PIDController m_distanceController;

  // Alignment tolerances
  private static final double POSITION_TOLERANCE_METERS = 0.05;  // 5cm
  private static final double HEADING_TOLERANCE_DEGREES = 2.0;

  // PID controller gains
  private static final double HEADING_P = 0.5;
  private static final double HEADING_I = 0.0;
  private static final double HEADING_D = 0.1;
  
  private static final double DISTANCE_P = 0.5;
  private static final double DISTANCE_I = 0.0;
  private static final double DISTANCE_D = 0.1;

  // Max speeds
  private static final double MAX_LINEAR_SPEED = 1.0;  // m/s
  private static final double MAX_ANGULAR_SPEED = 2.0;  // rad/s

  /**
   * Create a new QuestNavAlignCommand to navigate to a target pose
   * @param drive DriveSubsystem for robot movement
   * @param questNav QuestNav subsystem for pose tracking
   * @param targetPose The target Pose2d to navigate to
   */
  public QuestNavAlignCommand(DriveSubsystem drive, QuestNav questNav, Pose2d targetPose) {
    m_drive = drive;
    m_questNav = questNav;
    m_targetPose = targetPose;
    
    // Create heading controller (rotation to target heading)
    m_headingController = new PIDController(HEADING_P, HEADING_I, HEADING_D);
    m_headingController.setTolerance(HEADING_TOLERANCE_DEGREES);
    m_headingController.enableContinuousInput(-180, 180);
    
    // Create distance controller (drive toward target position)
    m_distanceController = new PIDController(DISTANCE_P, DISTANCE_I, DISTANCE_D);
    m_distanceController.setTolerance(POSITION_TOLERANCE_METERS);
    
    addRequirements(m_drive, m_questNav);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString(Constants.QuestNavConstants.kDashboardStatusKey, 
      "[QuestNavAlignCommand] Starting navigation to target pose...");
  }

  @Override
  public void execute() {
    // Check if QuestNav is tracking
    if (!m_questNav.hasValidPose()) {
      SmartDashboard.putString(Constants.QuestNavConstants.kDashboardStatusKey, 
        "[QuestNavAlignCommand] QuestNav not tracking!");
      m_drive.drive(0, 0, 0, false);
      return;
    }

    // Get current robot pose
    Pose2d currentPose = m_questNav.getRobotPose();
    
    // Calculate distance to target
    double distanceToTarget = currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
    
    // Calculate angle to target from current heading
    var toTarget = m_targetPose.getTranslation().minus(currentPose.getTranslation());
    double angleToTargetRad = Math.atan2(toTarget.getY(), toTarget.getX());
    double angleToTargetDeg = Math.toDegrees(angleToTargetRad);
    double currentHeadingDeg = currentPose.getRotation().getDegrees();
    
    // Calculate heading error
    double headingError = angleToTargetDeg - currentHeadingDeg;
    
    // Normalize heading error to [-180, 180]
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    
    // Calculate target heading (how much we need to rotate to face the target direction)
    double targetHeadingDeg = m_targetPose.getRotation().getDegrees();
    double rotationError = targetHeadingDeg - currentHeadingDeg;
    
    // Normalize rotation error
    while (rotationError > 180) rotationError -= 360;
    while (rotationError < -180) rotationError += 360;

    // Calculate velocity components
    // First, drive toward the target position
    double forwardSpeed = m_distanceController.calculate(distanceToTarget, 0.0);
    forwardSpeed = MathUtil.clamp(forwardSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    
    // Then, rotate to face the target heading
    double rotationSpeed = m_headingController.calculate(rotationError, 0.0);
    rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    
    // Drive using field-relative coordinates (x toward target position)
    // The angle for field-relative drive should point toward the target position
    double driveAngleRad = angleToTargetRad;
    
    // If close enough to target position, stop translating and just rotate to final heading
    if (distanceToTarget < POSITION_TOLERANCE_METERS) {
      forwardSpeed = 0;
    }
    
    // Send drive command with field-relative movement toward target
    // For swerve: drive(xSpeed, ySpeed, rot, fieldRelative)
    m_drive.drive(
      forwardSpeed * Math.cos(driveAngleRad),
      forwardSpeed * Math.sin(driveAngleRad),
      rotationSpeed,
      true  // Field relative
    );

    SmartDashboard.putString(Constants.QuestNavConstants.kDashboardStatusKey, 
      String.format("[QuestNavAlignCommand] Distance: %.2fm, Heading Error: %.1f°", 
        distanceToTarget, rotationError)
    );
  }

  @Override
  public boolean isFinished() {
    if (!m_questNav.hasValidPose()) {
      return false;
    }
    
    Pose2d currentPose = m_questNav.getRobotPose();
    
    // Check if within tolerance of target
    double distanceToTarget = currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
    
    double headingError = m_targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    
    boolean positionReached = distanceToTarget < POSITION_TOLERANCE_METERS;
    boolean headingReached = Math.abs(headingError) < HEADING_TOLERANCE_DEGREES;
    
    return positionReached && headingReached;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop robot movement
    m_drive.drive(0, 0, 0, false);
    
    if (interrupted) {
      SmartDashboard.putString(Constants.QuestNavConstants.kDashboardStatusKey, 
        "[QuestNavAlignCommand] Navigation interrupted");
    } else {
      SmartDashboard.putString(Constants.QuestNavConstants.kDashboardStatusKey, 
        "[QuestNavAlignCommand] Navigation complete - target reached");
    }
  }
}
