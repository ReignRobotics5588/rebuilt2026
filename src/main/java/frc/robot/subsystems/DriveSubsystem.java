package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DriveConstants;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

  private final SwerveDrive m_swerveDrive;
  private final Field2d m_field = new Field2d();
  private final StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Odometry/Robot", Pose2d.struct).publish();

  public DriveSubsystem() {
    SwerveDriveConfig driveConfig = new SwerveDriveConfig()
        .withSubsystem(this)
        .withTranslationController(new PIDController(1.0, 0.0, 0.0))
        .withRotationController(new PIDController(1.0, 0.0, 0.0))
        .withTelemetry("Swerve", TelemetryVerbosity.HIGH);

    SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve"));
    m_swerveDrive = SwerveParser.createSwerveDrive(driveConfig);

    SmartDashboard.putData("Field", m_field);
    setupPathPlanner();
  }

  private void setupPathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          m_swerveDrive::getPose,
          m_swerveDrive::resetOdometry,
          m_swerveDrive::getRobotRelativeSpeed,
          (speeds, feedforwards) -> m_swerveDrive.setRobotRelativeChassisSpeeds(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> DriverStation.getAlliance()
              .filter(a -> a == DriverStation.Alliance.Red).isPresent(),
          this);
    } catch (Exception e) {
      throw new RuntimeException("PathPlanner setup failed", e);
    }
  }

  @Override
  public void periodic() {
    m_swerveDrive.updateTelemetry();
    Pose2d pose = m_swerveDrive.getPose();
    m_field.setRobotPose(pose);
    m_posePublisher.set(pose);
  }

  @Override
  public void simulationPeriodic() {
    m_swerveDrive.simIterate();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xMs = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double yMs = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotRps = rot * DriveConstants.kMaxAngularSpeed;

    if (fieldRelative) {
      m_swerveDrive.setFieldRelativeChassisSpeeds(new ChassisSpeeds(xMs, yMs, rotRps));
    } else {
      m_swerveDrive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(xMs, yMs, rotRps));
    }
  }

  public void setX() {
    m_swerveDrive.lockPose();
  }

  public Pose2d getPose() {
    return m_swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    m_swerveDrive.resetOdometry(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeed() {
    return m_swerveDrive.getRobotRelativeSpeed();
  }

  public void zeroHeading() {
    m_swerveDrive.zeroGyro();
  }

  public double getHeading() {
    return m_swerveDrive.getGyroAngle().in(Units.Degrees);
  }
}
