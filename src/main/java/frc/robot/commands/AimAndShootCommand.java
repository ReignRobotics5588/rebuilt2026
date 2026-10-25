package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Aim-while-driving shot. The driver keeps control of translation while the robot
 * turns to face the alliance hub (using the vision-corrected pose), the flywheel
 * tracks the RPM for the current distance, and pieces feed once the flywheel is
 * up to speed and the robot is aimed.
 *
 * <p>Runs until interrupted, or in autos until {@link #finishAfterFeeding} is met.
 */
public class AimAndShootCommand extends Command {
  private final DriveSubsystem m_drive;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final Belt m_belt;
  private final DoubleSupplier m_xSpeed;
  private final DoubleSupplier m_ySpeed;
  private final PIDController m_headingController =
      new PIDController(DriveConstants.kAimP, 0.0, DriveConstants.kAimD);
  private double m_feedSecondsToFinish = Double.POSITIVE_INFINITY;
  private double m_fedSeconds = 0.0;

  /**
   * @param xSpeed forward speed as a fraction of max (field-relative), already shaped
   * @param ySpeed left speed as a fraction of max (field-relative), already shaped
   */
  public AimAndShootCommand(DriveSubsystem drive, Shooter shooter, Intake intake, Belt belt,
      DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    m_drive = drive;
    m_shooter = shooter;
    m_intake = intake;
    m_belt = belt;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_headingController.enableContinuousInput(-Math.PI, Math.PI);
    m_headingController.setTolerance(Math.toRadians(DriveConstants.kAimToleranceDegrees));
    addRequirements(drive, shooter, intake, belt);
  }

  /** Stationary version for autos. */
  public AimAndShootCommand(DriveSubsystem drive, Shooter shooter, Intake intake, Belt belt) {
    this(drive, shooter, intake, belt, () -> 0.0, () -> 0.0);
  }

  /** Finish once pieces have been fed for this long (for autos). */
  public AimAndShootCommand finishAfterFeeding(double seconds) {
    m_feedSecondsToFinish = seconds;
    return this;
  }

  @Override
  public void initialize() {
    m_fedSeconds = 0.0;
    m_headingController.reset();
    m_intake.setSpeed(Constants.IntakeConstants.kIntakeSpeed);
  }

  @Override
  public void execute() {
    Pose2d pose = m_drive.getPose();
    Translation2d toHub = FieldConstants.getAllianceHubCenter().minus(pose.getTranslation());
    Rotation2d targetHeading = toHub.getAngle().minus(ShooterConstants.kShooterFacing);

    double omega = m_headingController.calculate(
        pose.getRotation().getRadians(), targetHeading.getRadians());
    omega = MathUtil.clamp(omega, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed);
    m_drive.drive(m_xSpeed.getAsDouble(), m_ySpeed.getAsDouble(),
        omega / DriveConstants.kMaxAngularSpeed, true);

    double distance = toHub.getNorm();
    double targetRPM = Shooter.getRPMForDistance(distance);
    m_shooter.setFlywheelRPM(targetRPM);

    boolean aimed = m_headingController.atSetpoint();
    boolean feeding = aimed && m_shooter.isReadyToFeed(targetRPM);
    m_shooter.setIndexerSpeed(feeding ? ShooterConstants.kFeederSpeed : 0);
    m_belt.setSpeed(feeding ? Constants.BeltConstants.kBeltSpeed : 0);
    if (feeding) {
      m_fedSeconds += TimedRobot.kDefaultPeriod;
    }

    SmartDashboard.putBoolean("Shooter/Aimed", aimed);
    SmartDashboard.putBoolean("Shooter/FeederEnabled", feeding);
  }

  @Override
  public boolean isFinished() {
    return m_fedSeconds >= m_feedSecondsToFinish;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
    m_intake.setSpeed(0);
    m_shooter.setFlywheelRPM(0);
    m_shooter.setIndexerSpeed(0);
    m_belt.setSpeed(0);
    SmartDashboard.putBoolean("Shooter/FeederEnabled", false);
  }
}
