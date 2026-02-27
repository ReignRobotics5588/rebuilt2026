package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;

/**
 * Command that shoots with both shooter motors and the belt.
 * The shooter motors ramp to speed first, then the belt engages after the shooter reaches target RPM.
 * All motors stop when the command ends or is interrupted.
 */
public class ShooterBeltCommand extends Command {
  private final Shooter m_shooter;
  private final Belt m_belt;
  private final double kShooterRpmTolerance = Constants.ShooterConstants.kShooterRpmTolerance;
  private boolean m_beltStarted = false;

  /**
   * Creates a new ShooterBeltCommand.
   *
   * @param shooter the shooter subsystem (contains both flex and max motors)
   * @param belt the belt subsystem
   */
  public ShooterBeltCommand(Shooter shooter, Belt belt) {
    m_shooter = shooter;
    m_belt = belt;
    addRequirements(m_shooter, m_belt);
  }

  @Override
  public void initialize() {
    // Start flywheel using RPM control for consistent speed
    m_shooter.setShooterFlexRPM(Constants.ShooterConstants.kShooterTargetRPM);
    // Start feeder wheel at fixed percent output
    m_shooter.setShooterMaxSpeed(Constants.ShooterConstants.kFeederSpeed);
    m_beltStarted = false;
  }

  @Override
  public void execute() {
    // Check if shooter has reached target RPM
    if (!m_beltStarted
        && m_shooter.isAtTargetRPM(
            Constants.ShooterConstants.kShooterTargetRPM, kShooterRpmTolerance)) {
      // Once shooter is at speed, engage the belt
      m_belt.setSpeed(Constants.BeltConstants.kBeltSpeed);
      m_beltStarted = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all motors using percent output (0 RPM via setSpeed)
    m_shooter.setShooterFlexSpeed(0);
    m_shooter.setShooterMaxSpeed(0);
    m_belt.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted or cancelled
  }
}
