package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Simple command that drives the robot forward/backward a fixed distance (meters)
 * using odometry to determine completion. This is an open-loop drive (constant
 * speed) that stops when the Euclidean distance from the start pose exceeds
 * the requested distance.
 */
public class DriveDistanceCommand extends Command {
  private final DriveSubsystem m_drive;
  private final double m_distanceMeters;
  private final double m_speed;
  private double[] m_startXY = new double[2];

  /**
   * @param drive the DriveSubsystem
   * @param distanceMeters positive = forward, negative = backward
   * @param speed normalized speed magnitude (0..1). Direction is inferred from distance sign.
   */
  public DriveDistanceCommand(DriveSubsystem drive, double distanceMeters, double speed) {
    m_drive = drive;
    m_distanceMeters = Math.abs(distanceMeters);
    m_speed = Math.abs(speed) * (distanceMeters < 0 ? -1.0 : 1.0);
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    var p = m_drive.getPose();
    m_startXY[0] = p.getX();
    m_startXY[1] = p.getY();
  }

  @Override
  public void execute() {
    // Drive in robot-relative X (forward/back) direction
    m_drive.drive(m_speed, 0.0, 0.0, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
  }

  @Override
  public boolean isFinished() {
    var p = m_drive.getPose();
    double dx = p.getX() - m_startXY[0];
    double dy = p.getY() - m_startXY[1];
    double dist = Math.hypot(dx, dy);
    return dist >= m_distanceMeters;
  }
}
