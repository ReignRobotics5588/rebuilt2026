package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.io.DriveIO;
import frc.robot.subsystems.io.DriveIOInputsAutoLogged;

public class DriveSubsystem extends SubsystemBase {

  private final DriveIO m_io;
  private final DriveIOInputsAutoLogged m_inputs = new DriveIOInputsAutoLogged();
  private final Field2d m_field = new Field2d();
  private final StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/Robot", Pose2d.struct).publish();

  private final SysIdRoutine m_sysId;

  public DriveSubsystem(DriveIO io) {
    m_io = io;
    m_io.init(this);
    SmartDashboard.putData("Field", m_field);

    // Data is recorded by AdvantageKit (Drive/*); load the .wpilog in the SysId tool
    m_sysId = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null,
            state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(volts -> m_io.runDriveCharacterization(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Drive", m_inputs);

    Pose2d pose = getPose();
    m_field.setRobotPose(pose);
    m_posePublisher.set(pose);
  }

  @Override
  public void simulationPeriodic() {
    m_io.simIterate();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    m_io.drive(
        xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
        ySpeed * DriveConstants.kMaxSpeedMetersPerSecond,
        rot * DriveConstants.kMaxAngularSpeed,
        fieldRelative);
  }

  public void setX() {
    m_io.setX();
  }

  public Pose2d getPose() {
    double[] p = m_inputs.pose;
    return new Pose2d(p[0], p[1], Rotation2d.fromDegrees(p[2]));
  }

  public ChassisSpeeds getRobotRelativeSpeed() {
    return new ChassisSpeeds(
        m_inputs.chassisVxMetersPerSec,
        m_inputs.chassisVyMetersPerSec,
        m_inputs.chassisOmegaRadPerSec);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds,
      Matrix<N3, N1> stdDevs) {
    m_io.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  public void zeroHeading() {
    m_io.zeroHeading();
  }

  /** Drives straight forward/backward; give the robot several meters of room. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysId.dynamic(direction);
  }
}
