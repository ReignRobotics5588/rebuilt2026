package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Command to test the Shooter PID control system.
 * 
 * Reads the target RPM from the dashboard and uses the PID-controlled setFlywheelRPM() method
 * to maintain the flywheel at that speed. Useful for tuning P/I/D gains.
 * 
 * The command runs continuously until interrupted or cancelled.
 */
public class ShooterPIDTestCommand extends Command {
  private final Shooter m_shooter;

  /**
   * Creates a new ShooterPIDTestCommand.
   *
   * @param shooter the shooter subsystem to control
   */
  public ShooterPIDTestCommand(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    // Command started - no dashboard logging needed (status handled by subsystem telemetry)
  }

  @Override
  public void execute() {
    // Use the m_dashboardTargetRPM that's kept in sync by RobotContainer.periodic()
    m_shooter.setFlywheelRPM(m_shooter.getDashboardTargetRPM());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the flywheel when the command ends
    m_shooter.setFlywheelSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    // This command runs until explicitly cancelled or interrupted
    return false;
  }
}
