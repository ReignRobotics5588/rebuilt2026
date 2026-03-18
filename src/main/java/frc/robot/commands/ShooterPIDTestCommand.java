package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Command to test the Shooter PID control system.
 * 
 * The target RPM and PID gains are managed by RobotContainer.periodic(),
 * which reads dashboard values and applies them via Shooter.updatePIDFromDashboard().
 * This command simply keeps the shooter subsystem active while running.
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
    // Command started - RPM and PID values are managed by RobotContainer.periodic()
  }

  @Override
  public void execute() {
    // Target RPM and PID gains are updated by RobotContainer.periodic()
    // This command just maintains the requirement and allows the subsystem to run
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
