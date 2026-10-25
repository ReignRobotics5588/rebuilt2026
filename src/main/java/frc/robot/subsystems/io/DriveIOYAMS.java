package frc.robot.subsystems.io;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorController;

public class DriveIOYAMS implements DriveIO {

  private SwerveDrive m_swerveDrive;

  @Override
  public void init(SubsystemBase subsystem) {
    SwerveDriveConfig driveConfig = new SwerveDriveConfig()
        .withSubsystem(subsystem)
        .withTranslationController(new PIDController(1.0, 0.0, 0.0))
        .withRotationController(new PIDController(1.0, 0.0, 0.0))
        .withTelemetry("Swerve", Constants.kTelemetryVerbosity);

    SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve"));
    m_swerveDrive = SwerveParser.createSwerveDrive(driveConfig);

    setupPathPlanner(subsystem);
  }

  private void setupPathPlanner(SubsystemBase subsystem) {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          m_swerveDrive::getPose,
          m_swerveDrive::resetOdometry,
          m_swerveDrive::getRobotRelativeSpeed,
          (speeds, feedforwards) -> m_swerveDrive.setRobotRelativeChassisSpeeds(speeds, feedforwards.linearForces()),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> DriverStation.getAlliance()
              .filter(a -> a == DriverStation.Alliance.Red).isPresent(),
          subsystem);
    } catch (Exception e) {
      throw new RuntimeException("PathPlanner setup failed", e);
    }
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    m_swerveDrive.updateTelemetry();
    Pose2d pose = m_swerveDrive.getPose();
    inputs.pose = new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees()
    };
    ChassisSpeeds speeds = m_swerveDrive.getRobotRelativeSpeed();
    inputs.chassisVxMetersPerSec = speeds.vxMetersPerSecond;
    inputs.chassisVyMetersPerSec = speeds.vyMetersPerSecond;
    inputs.chassisOmegaRadPerSec = speeds.omegaRadiansPerSecond;
    inputs.gyroYawDegrees = m_swerveDrive.getGyroAngle().in(Units.Degrees);

    SwerveModule[] modules = m_swerveDrive.getModules();
    for (int i = 0; i < modules.length; i++) {
      SmartMotorController driveMotor = modules[i].getDriveMotorController();
      inputs.drivePositionsMeters[i] = driveMotor.getMeasurementPosition().in(Units.Meters);
      inputs.driveVelocitiesMetersPerSec[i] = driveMotor.getMeasurementVelocity().in(Units.MetersPerSecond);
      inputs.driveAppliedVolts[i] = driveMotor.getVoltage().in(Units.Volts);
    }
  }

  @Override
  public void runDriveCharacterization(double volts) {
    for (SwerveModule module : m_swerveDrive.getModules()) {
      module.getAzimuthMotorController().setPosition(Units.Degrees.of(0));
      module.getDriveMotorController().setVoltage(Units.Volts.of(volts));
    }
  }

  @Override
  public void drive(double xMetersPerSec, double yMetersPerSec, double rotRadPerSec,
      boolean fieldRelative) {
    ChassisSpeeds speeds = new ChassisSpeeds(xMetersPerSec, yMetersPerSec, rotRadPerSec);
    if (fieldRelative) {
      m_swerveDrive.setFieldRelativeChassisSpeeds(speeds);
    } else {
      m_swerveDrive.setRobotRelativeChassisSpeeds(speeds);
    }
  }

  @Override
  public void setX() {
    m_swerveDrive.lockPose();
  }

  @Override
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds,
      Matrix<N3, N1> stdDevs) {
    m_swerveDrive.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  @Override
  public void zeroHeading() {
    m_swerveDrive.zeroGyro();
  }

  @Override
  public void simIterate() {
    m_swerveDrive.simIterate();
  }
}
