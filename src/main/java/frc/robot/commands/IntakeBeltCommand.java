package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;

/**
 * Command that runs the intake and belt together simultaneously.
 * Both motors engage at the same time and stop when the command ends.
 */
public class IntakeBeltCommand extends Command {
  private final Intake m_intake;
  private final Belt m_belt;

  /**
   * Creates a new IntakeBeltCommand.
   *
   * @param intake the intake subsystem
   * @param belt the belt subsystem
   */
  public IntakeBeltCommand(Intake intake, Belt belt) {
    m_intake = intake;
    m_belt = belt;
    addRequirements(m_intake, m_belt);
  }

  @Override
  public void initialize() {
    m_intake.setSpeed(Constants.IntakeConstants.kIntakeSpeed);
    m_belt.setSpeed(Constants.BeltConstants.kBeltSpeed);
    SmartDashboard.putBoolean("Intake On", true);
  }

  @Override
  public void execute() {
    // Motors continue running from initialize
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setSpeed(0);
    m_belt.setSpeed(0);
    SmartDashboard.putBoolean("Intake On", false);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted or cancelled
  }
}
