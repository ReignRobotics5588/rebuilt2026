// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * Swerve hardware (CAN IDs, gearing, encoder offsets, module locations) lives in
 * src/main/deploy/swerve/*.json, and PathPlanner robot settings in
 * src/main/deploy/pathplanner/settings.json.
 */
public final class Constants {
  // YAMS telemetry detail. HIGH publishes every motor signal over NetworkTables,
  // which costs CAN and loop time on the roboRIO; switch to HIGH only while debugging.
  public static final TelemetryVerbosity kTelemetryVerbosity = TelemetryVerbosity.LOW;

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    // NEO free speed with 13T pinion is ~4.46 m/s
    public static final double kMaxSpeedMetersPerSecond = 4.4;
    public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

    // Aim-while-driving heading controller (output rad/s per rad of heading error)
    public static final double kAimP = 5.0;
    public static final double kAimD = 0.0;
    public static final double kAimToleranceDegrees = 3.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    // PathPlanner NamedCommands. Names must match the .auto/.path files exactly
    // (checked by DeployConfigTest).
    public static final String kIntakeOn = "IntakeOn";
    public static final String kIntakeOff = "IntakeOff";
    public static final String kBeltOn = "BeltOn";
    public static final String kBeltOff = "BeltOff";
    public static final String kShootAuto = "ShootAuto";
    public static final String kAimAndShoot = "AimAndShoot";
    public static final String kLimelightAlign = "LimelightAlign";
    public static final List<String> kNamedCommands = List.of(
        kIntakeOn, kIntakeOff, kBeltOn, kBeltOff, kShootAuto, kAimAndShoot, kLimelightAlign);

    // Auto shots end after feeding this long; the timeout is a safety cap in case
    // the flywheel never reaches speed or the robot never aims
    public static final double kShootFeedSeconds = 1.5;
    public static final double kShootTimeoutSeconds = 6.0;
  }

  public static final class IntakeConstants {
    public static final int kCanId = 15;
    public static final double kIntakeSpeed = -1;
    // Reverse ("puke") speed to spit game pieces back out
    public static final double kEjectSpeed = 0.7;
  }

  public static final class BeltConstants {
    public static final int kCanId = 12;
    public static final double kBeltSpeed = -0.35;
  }

  public static final class ShooterConstants {
    public static final int kFlywheelCanId = 17;
    public static final int kIndexerCanId = 14;

    // Feeder wheel speed (percent output)
    public static final double kFeederSpeed = 0.75;

    // Start feeding once the flywheel is within this many RPM below target
    public static final double kFeedRpmMargin = 350;

    public static final double kShooterAutoRPM = 2800;
    public static final double kShooterCloseRPM = 3200;

    // Flywheel RPM by distance (meters) from robot center to hub center.
    // PLACEHOLDER points from the old fixed shots; replace with measured shots
    // (the dashboard shows "Shooter/Hub Distance (m)"). Values outside the table
    // clamp to the nearest end.
    public static final InterpolatingDoubleTreeMap kDistanceToRPM = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.2, kShooterAutoRPM),
        Map.entry(2.5, kShooterCloseRPM));

    // Direction the shooter fires relative to the robot front. Use
    // Rotation2d.k180deg if it shoots out the back.
    public static final Rotation2d kShooterFacing = Rotation2d.kZero;

    // PID tuning parameters for SparkFlex internal controller (flywheel only).
    // YAMS runs the Spark in rotations per second (not RPM), so these are the
    // Bonney Lake RPM gains (P 0.0016869, D 0.0012) multiplied by 60.
    public static final double kFlywheelP = 0.1012;
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 0.072;
    // Volts per RPS: theoretical Neo Vortex kV = 12V / (6784 RPM / 60). Characterize on robot.
    public static final double kFlywheelFF = 0.1061;

    public static final int kFlywheelCurrentLimit = 60;
  }

  public static final class LimelightConstants {
    // Network table name for Limelight 2
    public static final String kLimelightTableName = "limelight";

    // Limelight 2 pipeline ID (0 = default April tag pipeline)
    public static final int kAprilTagPipeline = 0;

    // Camera mounting pose is configured in the Limelight web UI (robot-space camera pose)

    // Robot heading correction (PID gains for auto-alignment to AprilTag)
    public static final double kHeadingAlignP = 0.1;
    public static final double kHeadingAlignI = 0.0;
    public static final double kHeadingAlignD = 0.05;

    // Target alignment tolerance (degrees)
    public static final double kHeadingTolerance = 2.0;

    // Max angular velocity during alignment (rad/s)
    public static final double kMaxAlignmentAngularVelocity = 2.0;

    // Dashboard keys for Limelight telemetry
    public static final String kDashboardPrefix = "Limelight/";
    public static final String kDashboardTargetTagIdKey = kDashboardPrefix + "Target Tag ID";
    public static final String kDashboardHasTargetKey = kDashboardPrefix + "Has Target";
    public static final String kDashboardOffsetXKey = kDashboardPrefix + "Offset X (deg)";
    public static final String kDashboardOffsetYKey = kDashboardPrefix + "Offset Y (deg)";
    public static final String kDashboardTargetAreaKey = kDashboardPrefix + "Target Area (%)";
    public static final String kDashboardDistanceKey = kDashboardPrefix + "Distance to Target (m)";
    public static final String kDashboardHorizontalDistanceKey = kDashboardPrefix + "Horizontal Distance (m)";

    // General status/debug keys for Limelight
    public static final String kDashboardStatusKey = kDashboardPrefix + "Status";
    public static final String kDashboardDebugKey = kDashboardPrefix + "Debug";

    // Alliance-specific AprilTag IDs (change here if field targets differ)
    public static final int kRedAllianceTargetTagID = 10;
    public static final int kBlueAllianceTargetTagID = 26;
  }
}
