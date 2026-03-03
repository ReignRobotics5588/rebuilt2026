package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;

/**
 * Simple turn command that rotates the robot by a relative angle (degrees).
 * Uses a proportional controller and finishes when within a configurable tolerance.
 */
public class TurnToAngleCommand extends Command {
  private final DriveSubsystem m_drive;
  private final double m_relativeDegrees;
  private final double m_toleranceDeg;
  private final double m_kP;
  private double m_targetHeading;
  private double m_startHeading;

  /**
   * @param drive the DriveSubsystem
   * @param relativeDegrees positive = left turn, negative = right turn
   */
  public TurnToAngleCommand(DriveSubsystem drive, double relativeDegrees) {
    this(drive, relativeDegrees, 3.0, 0.01);
  }

  public TurnToAngleCommand(DriveSubsystem drive, double relativeDegrees, double toleranceDeg, double kP) {
    m_drive = drive;
    m_relativeDegrees = relativeDegrees;
    m_toleranceDeg = toleranceDeg;
    m_kP = kP;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_startHeading = m_drive.getHeading();
    m_targetHeading = m_startHeading + m_relativeDegrees;
  }

  @Override
  public void execute() {
    double current = m_drive.getHeading();
    double error = m_targetHeading - current;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    double rot = MathUtil.clamp(error * m_kP, -0.6, 0.6);
    m_drive.drive(0.0, 0.0, rot, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
  }

  @Override
  public boolean isFinished() {
    double current = m_drive.getHeading();
    double error = m_targetHeading - current;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    return Math.abs(error) <= m_toleranceDeg;
  }
}
