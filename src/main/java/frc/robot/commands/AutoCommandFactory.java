package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Factory class for creating autonomous command sequences.
 * 
 * This class provides static factory methods that build complex command sequences
 * for autonomous routines by combining individual commands.
 * 
 * Example usage:
 * Command autoCommand = AutoCommandFactory.shootAndIntakeSequence(shooter, belt, intake);
 */
public class AutoCommandFactory {

  /**
   * Creates a sequence that shoots, then intakes.
   * Useful for scoring game pieces then intaking the next one.
   *
   * @param shooter the shooter subsystem
   * @param belt the belt subsystem
   * @param intake the intake subsystem
   * @return a command that shoots, waits 1 second, then starts intaking
   */
  public static Command shootThenIntake(Shooter shooter, Belt belt, Intake intake) {
    return Commands.sequence(
        new ShooterBeltCommand(shooter, belt),
        Commands.waitSeconds(1.0),
        new IntakeBeltCommand(intake, belt)
    );
  }

  /**
   * Creates a sequence that intakes multiple pieces with delays.
   *
   * @param intake the intake subsystem
   * @param belt the belt subsystem
   * @param numPieces the number of pieces to intake
   * @return a command that intakes multiple pieces with 0.5 second delays between
   */
  public static Command intakeMultiple(Intake intake, Belt belt, int numPieces) {
    Command sequence = Commands.none();
    for (int i = 0; i < numPieces; i++) {
      if (i > 0) {
        sequence = sequence.andThen(Commands.waitSeconds(0.5));
      }
      sequence = sequence.andThen(new IntakeBeltCommand(intake, belt));
    }
    return sequence;
  }

  /**
   * Creates a parallel command that shoots while the intake operates independently.
   * Shooter waits for target RPM before engaging belt.
   *
   * @param shooter the shooter subsystem
   * @param belt the belt subsystem
   * @param intake the intake subsystem
   * @return a command that shoots and intakes in parallel
   */
  public static Command shootWhileIntaking(Shooter shooter, Belt belt, Intake intake) {
    return Commands.parallel(
        new ShooterBeltCommand(shooter, belt),
        new IntakeBeltCommand(intake, belt)
    );
  }
}
