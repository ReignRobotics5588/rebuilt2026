package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    //m_shooter.setFlywheelRPM(Constants.ShooterConstants.kShooterTargetRPM);
    m_shooter.setFlywheelSpeed(0.75);
    m_beltStarted = false;
  }

  @Override
  public void execute() {
    // Gate feeding: use hysteresis to avoid rapid on/off if RPM briefly dips.
    double target = Constants.ShooterConstants.kShooterTargetRPM;
    double tolerance = kShooterRpmTolerance;
    double hysteresis = Constants.ShooterConstants.kShooterRpmHysteresis;

    double rpmError = Math.abs(m_shooter.getFlywheelRPM() - target);

    // Start feeding when within tolerance. Stop only when error exceeds tolerance + hysteresis.
    if (!m_beltStarted && rpmError <= tolerance) {
      // Flywheel is up to speed — enable feeder and belt
      m_shooter.setIndexerSpeed(Constants.ShooterConstants.kFeederSpeed);
      m_belt.setSpeed(Constants.BeltConstants.kBeltSpeed);
      m_beltStarted = true;
    } else if (m_beltStarted && rpmError > (tolerance + hysteresis)) {
      // Flywheel dropped well below target — stop feeding to avoid jams/misfires
      m_shooter.setIndexerSpeed(0);
      m_belt.setSpeed(0);
      m_beltStarted = false;
    }

    // Publish feeder state for dashboard/driver visibility
    SmartDashboard.putBoolean("Shooter/FeederEnabled", m_beltStarted);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all motors using percent output (0 RPM via setSpeed)
    m_shooter.setFlywheelSpeed(0);
    m_shooter.setIndexerSpeed(0);
    m_belt.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted or cancelled
  }
}
