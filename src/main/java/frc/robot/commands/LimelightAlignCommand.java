package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

/**
 * Command to automatically align robot heading to detected April tag using Limelight vision.
 * 
 * Runs PID controller on the horizontal offset (tx) from Limelight to rotate robot
 * until it's facing the April tag. Useful for scoring alignment in autonomous or teleop.
 * 
 * The command continues until the heading error is within tolerance or until interrupted.
 */
public class LimelightAlignCommand extends Command {
  private final DriveSubsystem m_drive;
  private final Limelight m_limelight;
  private final PIDController m_headingController;

  /**
   * Create a new LimelightAlignCommand
   * @param drive DriveSubsystem for robot movement
   * @param limelight Limelight vision subsystem
   */
  public LimelightAlignCommand(DriveSubsystem drive, Limelight limelight) {
    m_drive = drive;
    m_limelight = limelight;
    
    // Create PID controller for heading alignment
    m_headingController = new PIDController(
        Constants.LimelightConstants.kHeadingAlignP,
        Constants.LimelightConstants.kHeadingAlignI,
        Constants.LimelightConstants.kHeadingAlignD
    );
    
    // Set tolerance for alignment completion
    m_headingController.setTolerance(Constants.LimelightConstants.kHeadingTolerance);
    
    addRequirements(m_drive, m_limelight);
  }

  @Override
  public void initialize() {
    // Ensure Limelight is active
    m_limelight.setLimelightActive(true);
    SmartDashboard.putString(Constants.LimelightConstants.kDashboardStatusKey, "[LimelightAlignCommand] Starting alignment...");
  }

  @Override
  public void execute() {
    if (!m_limelight.hasDesiredTarget()) {
      if (!m_limelight.hasValidTarget()) {
        SmartDashboard.putString(Constants.LimelightConstants.kDashboardStatusKey, "[LimelightAlignCommand] No April tag detected!");
      } else {
        SmartDashboard.putString(Constants.LimelightConstants.kDashboardStatusKey, "[LimelightAlignCommand] Wrong tag detected! ID: " + m_limelight.getTargetID() + ", Desired: " + m_limelight.getDesiredTagID());
      }
      m_drive.drive(0, 0, 0, false);
      return;
    }

    // Get horizontal offset to target (positive = target to the right)
    double offsetX = m_limelight.getTargetOffsetX();
    
    // Calculate rotation speed using PID
    // tx = 0 means we're aligned
    double rotationSpeed = m_headingController.calculate(offsetX, 0.0);
    
    // Clamp rotation speed to maximum allowed angular velocity
    rotationSpeed = MathUtil.clamp(
        rotationSpeed,
        -Constants.LimelightConstants.kMaxAlignmentAngularVelocity,
        Constants.LimelightConstants.kMaxAlignmentAngularVelocity
    );
    
    // Drive command: no translation, only rotation
    m_drive.drive(0, 0, rotationSpeed, false);
    
  SmartDashboard.putString(Constants.LimelightConstants.kDashboardDebugKey, String.format(
    "[LimelightAlignCommand] Offset: %.2f° | Rotation: %.2f rad/s | %s",
    offsetX,
    rotationSpeed,
    m_limelight.getDebugString()
  ));
  }

  @Override
  public void end(boolean interrupted) {
    // Stop robot rotation when done
    m_drive.drive(0, 0, 0, false);
    
    if (interrupted) {
      SmartDashboard.putString(Constants.LimelightConstants.kDashboardStatusKey, "[LimelightAlignCommand] Alignment interrupted");
    } else {
      SmartDashboard.putString(Constants.LimelightConstants.kDashboardStatusKey, "[LimelightAlignCommand] Alignment complete");
    }
  }

  @Override
  public boolean isFinished() {
    // Finish when heading error is within tolerance and we have the desired tag
    if (!m_limelight.hasDesiredTarget()) {
      return false;  // Keep running to find desired target
    }
    
    double offsetX = m_limelight.getTargetOffsetX();
    boolean atTarget = Math.abs(offsetX) <= Constants.LimelightConstants.kHeadingTolerance;
    
    return atTarget && m_headingController.atSetpoint();
  }
}
