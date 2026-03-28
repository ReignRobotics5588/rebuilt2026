package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.QuestNav;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

    // Mutable holders so lambdas can capture "by reference"
    final double[] startXY = new double[2];
    final double[] startHeading = new double[1];
    final double[] targetHeading = new double[1];

    // Step A: capture starting pose/heading
    Command captureStart = Commands.runOnce(() -> {
      var p = drive.getPose();
      startXY[0] = p.getX();
      startXY[1] = p.getY();
      startHeading[0] = drive.getHeading();
      targetHeading[0] = startHeading[0] + leftTurnDegrees;
    });

    // Step B: drive backwards until we've moved ~1.5 meters from start
    Command driveBack = Commands.run(() -> {
      // Drive backwards at 30% of max speed
      drive.drive(-0.3, 0, 0, false);
    }, drive)
        .until(() -> {
          var p = drive.getPose();
          double dx = p.getX() - startXY[0];
          double dy = p.getY() - startXY[1];
          double dist = Math.hypot(dx, dy);
          return dist >= 1.5; // meters
        })
        .finallyDo(() -> drive.drive(0, 0, 0, false));

    // Step C: turn left by leftTurnDegrees (positive = left). Use a simple
    // proportional rotation and finish when within 3 degrees.
    Command turnLeft = Commands.run(() -> {
      double current = drive.getHeading();
      double targ = targetHeading[0];
      double error = targ - current;
      // Normalize to [-180, 180]
      while (error > 180) error -= 360;
      while (error < -180) error += 360;

      // Simple P controller for rotation command (clamped)
      double kP = 0.01; // conservative gain
      double rot = MathUtil.clamp(error * kP, -0.5, 0.5);
      drive.drive(0, 0, rot, false);
    }, drive)
        .until(() -> {
          double current = drive.getHeading();
          double targ = targetHeading[0];
          double error = targ - current;
          while (error > 180) error -= 360;
          while (error < -180) error += 360;
          return Math.abs(error) <= 3.0; // degrees tolerance
        })
        .finallyDo(() -> drive.drive(0, 0, 0, false));

    // Step D: fine align using Limelight (existing command)
    Command align = new LimelightAlignCommand(drive, limelight).withTimeout(3.0);

    // Step E: run shooter+belt until interrupted / timeout (shoot for 4s)
    Command shoot = new ShooterBeltCommand(shooter, belt).withTimeout(4.0);

    // Compose everything sequentially
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

  /**
   * Drive backwards a fixed distance (meters), turn left by a given angle (degrees),
   * navigate to target position using QuestNav full-field pose tracking, and then run the shooter+belt sequence.
   *
   * This method uses QuestNav's full-field pose estimation instead of vision alignment,
   * allowing autonomous navigation to any point on the field.
   *
   * @param drive DriveSubsystem instance
   * @param questNav QuestNav instance for full-field pose tracking
   * @param shooter Shooter instance
   * @param belt Belt instance
   * @param leftTurnDegrees How many degrees to turn left (positive = left)
   * @return composed autonomous command
   */
  public static Command driveBackTurnAimShoot(
      DriveSubsystem drive,
      QuestNav questNav,
      Shooter shooter,
      Belt belt,
      double leftTurnDegrees) {

    // Mutable holders so lambdas can capture "by reference"
    final double[] startXY = new double[2];
    final double[] startHeading = new double[1];
    final double[] targetHeading = new double[1];
    final Pose2d[] targetPose = new Pose2d[1];

    // Step A: capture starting pose/heading
    Command captureStart = Commands.runOnce(() -> {
      var p = drive.getPose();
      startXY[0] = p.getX();
      startXY[1] = p.getY();
      startHeading[0] = drive.getHeading();
      targetHeading[0] = startHeading[0] + leftTurnDegrees;
      
      // Set target pose to a position that is 1.5m backward from current position
      // and rotated by leftTurnDegrees
      double backDist = 1.5;
      double newX = startXY[0] - backDist * Math.cos(Math.toRadians(startHeading[0]));
      double newY = startXY[1] - backDist * Math.sin(Math.toRadians(startHeading[0]));
      targetPose[0] = new Pose2d(newX, newY, Rotation2d.fromDegrees(targetHeading[0]));
    });

    // Step B: drive backwards until we've moved ~1.5 meters from start
    Command driveBack = Commands.run(() -> {
      // Drive backwards at 30% of max speed
      drive.drive(-0.3, 0, 0, false);
    }, drive)
        .until(() -> {
          var p = drive.getPose();
          double dx = p.getX() - startXY[0];
          double dy = p.getY() - startXY[1];
          double dist = Math.hypot(dx, dy);
          return dist >= 1.5; // meters
        })
        .finallyDo(() -> drive.drive(0, 0, 0, false));

    // Step C: navigate to target pose using QuestNav full-field tracking
    Command navigateToTarget = new QuestNavAlignCommand(drive, questNav, targetPose[0]).withTimeout(5.0);

    // Step D: run shooter+belt until interrupted / timeout (shoot for 4s)
    Command shoot = new ShooterBeltCommand(shooter, belt).withTimeout(4.0);

    // Compose everything sequentially
    return Commands.sequence(
        captureStart,
        driveBack,
        navigateToTarget,
        Commands.waitSeconds(0.15),
        shoot,
        Commands.runOnce(() -> drive.drive(0, 0, 0, false))
    );
  }
}
