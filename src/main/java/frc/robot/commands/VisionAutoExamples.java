package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Example autonomous commands showing how to use Limelight vision for positioning.
 * 
 * These are templates - customize based on your specific game strategy.
 */
public class VisionAutoExamples {

  /**
   * Simple example: Align to nearest AprilTag
   * 
   * Usage in getAutonomousCommand():
   * return VisionAutoExamples.alignToTag(m_robotDrive, m_limelight);
   */
  public static Command alignToTag(DriveSubsystem drive, Limelight limelight) {
    return Commands.sequence(
        // First, align to AprilTag
        new LimelightAlignCommand(drive, limelight),
        // Then stop
        Commands.runOnce(() -> drive.drive(0, 0, 0, false))
    );
  }

  /**
   * Example: Drive forward to specific distance from AprilTag
   * 
   * This is a basic example - you would need to implement distance tracking
   */
  public static Command driveToTag(DriveSubsystem drive, Limelight limelight, double targetDistance) {
    return Commands.run(() -> {
      if (limelight.hasValidTarget()) {
        // Get current distance
        double currentDistance = limelight.getDistanceToTarget();
        
        // Simple proportional control for forward/backward
        double error = targetDistance - currentDistance;
        double driveSpeed = Math.max(-0.5, Math.min(0.5, error * 0.5));  // Clamp to ±0.5
        
        // Get alignment error
        double alignmentError = limelight.getTargetOffsetX();
        double rotationSpeed = alignmentError * 0.05;  // Simple proportional rotation
        
        // Drive with both forward and rotation
        drive.drive(driveSpeed, 0, rotationSpeed, false);
      } else {
        // Stop if target lost
        drive.drive(0, 0, 0, false);
      }
    }, drive)
    .until(() -> {
      if (!limelight.hasValidTarget()) return true;
      
      // Finish when within tolerance (e.g., ±0.1m and aligned)
      double distance = limelight.getDistanceToTarget();
      double alignment = limelight.getTargetOffsetX();
      return Math.abs(distance - targetDistance) < 0.1 && Math.abs(alignment) < 2.0;
    })
    .finallyDo(() -> drive.drive(0, 0, 0, false));
  }

  /**
   * Example: Shoot at AprilTag (requires shooter subsystem)
   * 
   * This demonstrates checking if robot is aligned before proceeding
   */
  public static Command shootAtTag(DriveSubsystem drive, Limelight limelight) {
    return Commands.sequence(
        // Step 1: Align to tag
        new LimelightAlignCommand(drive, limelight),
        
        // Step 2: Wait for alignment to stabilize (0.2 seconds)
        Commands.waitSeconds(0.2),
        
        // Step 3: Verify still aligned and target still visible
        Commands.runOnce(() -> {
          if (limelight.hasValidTarget() && Math.abs(limelight.getTargetOffsetX()) < 3.0) {
            SmartDashboard.putString("Vision/Status", "[VisionAuto] Aligned! Would shoot here.");
            // TODO: Add shooter.shoot() command here
          } else {
            SmartDashboard.putString("Vision/Status", "[VisionAuto] Lost target or misaligned!");
          }
        }),
        
        // Step 4: Stop
        Commands.runOnce(() -> drive.drive(0, 0, 0, false))
    );
  }

  /**
   * Example: Debug command to display Limelight data continuously
   */
  public static Command debugVisionData(Limelight limelight) {
    return Commands.run(() -> {
      if (limelight.hasValidTarget()) {
        SmartDashboard.putString("Vision/Debug", limelight.getDebugString());
        
        // Additional detailed output:
        double[] pose = limelight.getRobotPose();
        double[] poseRelative = limelight.getRobotPoseRelativeToTarget();
        
        String formatted = String.format(
            "Field Pose: (%.2f, %.2f) at %.1f° | Tag Relative: x=%.2fm, y=%.2fm",
            pose[0], pose[1], pose[5],
            poseRelative[0], poseRelative[1]
        );
        SmartDashboard.putString("Vision/DebugExtra", formatted);
      } else {
        SmartDashboard.putString("Vision/Status", "[Vision] No target detected");
      }
    }, limelight)
    .withTimeout(5.0);  // Run for 5 seconds
  }

  /**
   * Example: Multi-target sequence (shoot at multiple tags)
   * 
   * This shows how to use vision for game sequences
   */
  public static Command shootMultipleTags(
      DriveSubsystem drive, 
      Limelight limelight,
      int... tagIDs) {
    
    return Commands.sequence(
  // For each tag ID in the list
  Commands.runOnce(() -> SmartDashboard.putString("Vision/Status", "[VisionAuto] Starting multi-tag sequence")),
        
        // Shoot at each tag in sequence
        new LimelightAlignCommand(drive, limelight),
        Commands.waitSeconds(0.5),
  Commands.runOnce(() -> SmartDashboard.putString("Vision/Status", "[VisionAuto] Aligned to tag")),
        
        // Stop
        Commands.runOnce(() -> drive.drive(0, 0, 0, false))
    );
  }
}
