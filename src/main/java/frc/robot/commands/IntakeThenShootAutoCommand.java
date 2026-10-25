package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Belt;

/**
 * Auto command that:
 * 1. Runs the intake to collect game pieces
 * 2. Spins up the shooter flywheel
 * 3. Once the shooter RPM reaches the target, engages the indexer and belt to shoot
 * 
 * This command runs until explicitly interrupted.
 */
public class IntakeThenShootAutoCommand extends Command {
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Belt m_belt;
  private final double m_shooterRPMTarget;
  private boolean m_indexerStarted = false;

  /**
   * Creates a new IntakeThenShootAutoCommand.
   *
   * @param intake the intake subsystem
   * @param shooter the shooter subsystem
   * @param belt the belt subsystem
   * @param shooterRPMTarget flywheel target RPM; feeding starts within kFeedRpmMargin of it
   */
  public IntakeThenShootAutoCommand(Intake intake, Shooter shooter, Belt belt, double shooterRPMTarget) {
    m_intake = intake;
    m_shooter = shooter;
    m_belt = belt;
    m_shooterRPMTarget = shooterRPMTarget;
    addRequirements(m_intake, m_shooter, m_belt);
  }

  @Override
  public void initialize() {
    // Start the intake
    m_intake.setSpeed(Constants.IntakeConstants.kIntakeSpeed);
    
    // Start the flywheel using PID control to reach and maintain target RPM
    m_shooter.setFlywheelRPM(m_shooterRPMTarget);
    
    // Belt stays off until indexer starts
    m_belt.setSpeed(0);
    
    m_indexerStarted = false;
    SmartDashboard.putBoolean("Auto/Intake Started", true);
    SmartDashboard.putBoolean("Auto/Indexer Started", false);
  }

  @Override
  public void execute() {
    // Check if shooter has reached target RPM to start indexing and belt
    if (m_shooter.isReadyToFeed(m_shooterRPMTarget)) {
      // Flywheel is up to speed — enable indexer and belt together
      m_shooter.setIndexerSpeed(Constants.ShooterConstants.kFeederSpeed);
      m_belt.setSpeed(Constants.BeltConstants.kBeltSpeed);
      m_indexerStarted = true;
    } else {
      // Flywheel dipped — stop both so the belt doesn't jam pieces into a stopped indexer
      m_shooter.setIndexerSpeed(0);
      m_belt.setSpeed(0);
      m_indexerStarted = false;
    }
    SmartDashboard.putBoolean("Auto/Indexer Started", m_indexerStarted);
    
    // Update dashboard with current RPM
    SmartDashboard.putNumber("Auto/Current Flywheel RPM", m_shooter.getFlywheelRPM());
    SmartDashboard.putNumber("Auto/Target Flywheel RPM", m_shooterRPMTarget);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    m_intake.setSpeed(0);
    m_shooter.setFlywheelRPM(0);
    m_shooter.setIndexerSpeed(0);
    m_belt.setSpeed(0);
    
    SmartDashboard.putBoolean("Auto/Intake Started", false);
    SmartDashboard.putBoolean("Auto/Indexer Started", false);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted or cancelled
  }
}
