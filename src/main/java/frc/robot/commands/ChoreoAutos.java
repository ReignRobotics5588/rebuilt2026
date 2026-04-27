package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Choreo autonomous routines. Each public method returns a Command for the auto chooser.
 *
 * <p>Trajectory files (.traj) live in src/main/deploy/choreo/ and are created in the Choreo app.
 * Trajectory names passed to routine.trajectory() must match the file names exactly.
 */
public class ChoreoAutos {

  private final AutoFactory factory;
  private final DriveSubsystem drive;

  public ChoreoAutos(DriveSubsystem drive) {
    this.drive = drive;

    PIDController xController = new PIDController(AutoConstants.kChoreoTranslationKP, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kChoreoTranslationKP, 0, 0);
    PIDController thetaController = new PIDController(AutoConstants.kChoreoRotationKP, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    factory = new AutoFactory(
        drive::getPose,
        drive::resetOdometry,
        (SwerveSample sample) -> {
          var pose = drive.getPose();
          drive.driveChassisSpeeds(new ChassisSpeeds(
              sample.vx + xController.calculate(pose.getX(), sample.x),
              sample.vy + yController.calculate(pose.getY(), sample.y),
              sample.omega + thetaController.calculate(
                  pose.getRotation().getRadians(), sample.heading)));
        },
        true,
        drive);
  }

  /**
   * Drive forward along a Choreo path. Good first test to verify path following works.
   * Create a path named "DriveForward" in the Choreo app.
   */
  public Command driveForward() {
    AutoRoutine routine = factory.newRoutine("DriveForward");
    AutoTrajectory traj = routine.trajectory("DriveForward");
    routine.active().onTrue(traj.resetOdometry().andThen(traj.cmd()));
    return routine.cmd();
  }

  /**
   * Drive back from starting position, then shoot the preloaded note.
   * Create a path named "DriveBackShoot" in the Choreo app.
   */
  public Command driveBackShoot(Shooter shooter, Belt belt) {
    AutoRoutine routine = factory.newRoutine("DriveBackShoot");
    AutoTrajectory traj = routine.trajectory("DriveBackShoot");
    routine.active().onTrue(
        traj.resetOdometry()
            .andThen(traj.cmd())
            .andThen(new ShooterBeltCommand(shooter, belt).withTimeout(4.0)));
    return routine.cmd();
  }

  /**
   * Drive a path while running intake, then shoot once the path finishes.
   * Create a path named "IntakePath" in the Choreo app.
   */
  public Command intakeThenShoot(Intake intake, Shooter shooter, Belt belt) {
    AutoRoutine routine = factory.newRoutine("IntakePath");
    AutoTrajectory traj = routine.trajectory("IntakePath");
    routine.active().onTrue(
        traj.resetOdometry()
            .andThen(Commands.parallel(
                traj.cmd(),
                new IntakeBeltCommand(intake, belt)))
            .andThen(new IntakeThenShootAutoCommand(
                intake, shooter, belt, ShooterConstants.kShooterAutoRPM)));
    return routine.cmd();
  }
}
