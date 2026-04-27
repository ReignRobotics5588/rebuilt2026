package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.AutoConstants;

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

  /**
   * Creates a command that runs the intake while simultaneously ramping up the shooter flywheel.
   * Once the shooter reaches the specified RPM target, the indexer and belt engage to feed game pieces.
   *
   * @param intake the intake subsystem
   * @param shooter the shooter subsystem
   * @param belt the belt subsystem
   * @param shooterRPMTarget the RPM threshold at which to start the indexer and belt (e.g., 1000)
   * @return a command that intakes then shoots once flywheel reaches target RPM
   */
  public static Command intakeThenShoot(Intake intake, Shooter shooter, Belt belt, double shooterRPMTarget) {
    return new IntakeThenShootAutoCommand(intake, shooter, belt, shooterRPMTarget);
  }

  /**
   * Creates a command that runs the intake while simultaneously ramping up the shooter flywheel.
   * Once the shooter reaches 1000 RPM (default), the indexer and belt engage to feed game pieces.
   *
   * @param intake the intake subsystem
   * @param shooter the shooter subsystem
   * @param belt the belt subsystem
   * @return a command that intakes then shoots once flywheel reaches 1000 RPM
   */
  public static Command intakeThenShoot(Intake intake, Shooter shooter, Belt belt) {
    return new IntakeThenShootAutoCommand(intake, shooter, belt);
  }

  /**
   * Drive backwards a fixed distance (meters), turn left by a given angle (degrees),
   * align to the target with Limelight, and then run the shooter+belt sequence.
   *
   * This method composes simple run/until commands so it does not require new
   * command classes. It assumes odometry via DriveSubsystem.getPose() is available.
   *
   * @param drive DriveSubsystem instance
   * @param limelight Limelight instance
   * @param shooter Shooter instance
   * @param belt Belt instance
   * @param leftTurnDegrees How many degrees to turn left (positive = left)
   * @return composed autonomous command
   */
  public static Command driveBackTurnAimShoot(
      DriveSubsystem drive,
      Limelight limelight,
      Shooter shooter,
      Belt belt,
      double leftTurnDegrees) {

    final double[] startXY = new double[2];

    Command captureStart = Commands.runOnce(() -> {
      var p = drive.getPose();
      startXY[0] = p.getX();
      startXY[1] = p.getY();
    });

    Command driveBack = Commands.run(
        () -> drive.drive(-AutoConstants.kAutoDriveBackSpeed, 0, 0, false), drive)
        .until(() -> {
          var p = drive.getPose();
          return Math.hypot(p.getX() - startXY[0], p.getY() - startXY[1])
              >= AutoConstants.kAutoDriveBackDistanceMeters;
        })
        .finallyDo(() -> drive.drive(0, 0, 0, false));

    Command turnLeft = new TurnToAngleCommand(
        drive, leftTurnDegrees, 3.0, AutoConstants.kAutoTurnKP);

    Command align = new LimelightAlignCommand(drive, limelight).withTimeout(3.0);

    Command shoot = new ShooterBeltCommand(shooter, belt).withTimeout(4.0);

    return Commands.sequence(
        captureStart,
        driveBack,
        turnLeft,
        align,
        Commands.waitSeconds(0.15),
        shoot,
        Commands.runOnce(() -> drive.drive(0, 0, 0, false))
    );
  }
}
