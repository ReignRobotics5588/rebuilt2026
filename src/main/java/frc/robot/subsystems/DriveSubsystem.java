package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final ModuleIO[] moduleIOs;
  private final ModuleIOInputsAutoLogged[] moduleInputs;

  private final SwerveDriveOdometry odometry;
  private final Field2d field = new Field2d();

  public DriveSubsystem(GyroIO gyroIO, ModuleIO... moduleIOs) {
    this.gyroIO = gyroIO;
    this.moduleIOs = moduleIOs;
    this.moduleInputs = new ModuleIOInputsAutoLogged[moduleIOs.length];
    for (int i = 0; i < moduleIOs.length; i++) {
      moduleInputs[i] = new ModuleIOInputsAutoLogged();
    }

    // Read initial inputs before constructing odometry
    for (int i = 0; i < moduleIOs.length; i++) {
      moduleIOs[i].updateInputs(moduleInputs[i]);
    }
    gyroIO.updateInputs(gyroInputs);

    odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        gyroInputs.yawPosition,
        getModulePositions());

    SmartDashboard.putData("Field", field);
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    // Update module sims/hardware first so their state reflects latest commands
    for (int i = 0; i < moduleIOs.length; i++) {
      moduleIOs[i].updateInputs(moduleInputs[i]);
      Logger.processInputs("Drive/Module" + i, moduleInputs[i]);
    }

    // In simulation, integrate chassis rotation from module states to drive the gyro sim
    if (gyroIO instanceof GyroIOSim) {
      ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
      Rotation2d newYaw = gyroInputs.yawPosition
          .plus(new Rotation2d(speeds.omegaRadiansPerSecond * 0.02));
      ((GyroIOSim) gyroIO).updateSimState(newYaw, speeds.omegaRadiansPerSecond);
    }

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    odometry.update(gyroInputs.yawPosition, getModulePositions());

    // Publish robot pose and module states for AdvantageScope visualization
    Logger.recordOutput("Drive/RobotPose", getPose());
    Logger.recordOutput("Drive/SwerveStates", getModuleStates());
    field.setRobotPose(getPose());
  }

  /** Drive with joystick-style inputs (normalized -1..1 speeds). */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double yDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xDelivered, yDelivered, rotDelivered,
                gyroInputs.yawPosition)
            : new ChassisSpeeds(xDelivered, yDelivered, rotDelivered));

    setModuleStates(states);
  }

  /** Lock wheels in X formation to prevent movement. */
  public void setX() {
    setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    });
  }

  /** Command all swerve modules to the given states. */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    Logger.recordOutput("Drive/DesiredStates", desiredStates);
    for (int i = 0; i < moduleIOs.length; i++) {
      // Optimize: avoid turning more than 90° by flipping direction if needed
      desiredStates[i].optimize(moduleInputs[i].turnAbsolutePosition);
      moduleIOs[i].setDriveVelocity(desiredStates[i].speedMetersPerSecond);
      moduleIOs[i].setTurnPosition(desiredStates[i].angle.getRadians());
    }
  }

  public void resetEncoders() {
    for (ModuleIO m : moduleIOs) m.resetEncoders();
  }

  public void zeroHeading() {
    gyroIO.reset();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyroInputs.yawPosition, getModulePositions(), pose);
  }

  @AutoLogOutput(key = "Drive/Pose")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getHeading() {
    return gyroInputs.yawPosition.getDegrees();
  }

  public double getTurnRate() {
    return Math.toDegrees(gyroInputs.yawVelocityRadPerSec);
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[moduleIOs.length];
    for (int i = 0; i < moduleIOs.length; i++) {
      positions[i] = new SwerveModulePosition(
          moduleInputs[i].drivePositionMeters,
          moduleInputs[i].turnAbsolutePosition);
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[moduleIOs.length];
    for (int i = 0; i < moduleIOs.length; i++) {
      states[i] = new SwerveModuleState(
          moduleInputs[i].driveVelocityMetersPerSec,
          moduleInputs[i].turnAbsolutePosition);
    }
    return states;
  }
}
