package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// BuiltInWidgets intentionally unused in this lightweight layout builder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

/**
 * Builds a Telemetry Shuffleboard tab with an elastic/list layout that
 * displays all of the keys and live values currently published by the robot.
 *
 * Call TelemetryLayout.setup(...) from RobotContainer after subsystems are
 * instantiated so the tab appears on the Driver Station Shuffleboard.
 */
public final class TelemetryLayout {
  private TelemetryLayout() {}

  public static void setup(
      DriveSubsystem drive,
      Limelight limelight,
      Shooter shooter,
      Belt belt,
      Intake intake) {

    ShuffleboardTab tab = Shuffleboard.getTab("Telemetry");

    // Limelight layout (elastic list)
    ShuffleboardLayout limel = tab.getLayout("Limelight", BuiltInLayouts.kList)
        .withSize(3, 7)
        .withPosition(0, 0);

    limel.addBoolean("Has Target", limelight::hasValidTarget);
    limel.addNumber("Target Offset X (deg)", limelight::getTargetOffsetX);
    limel.addNumber("Target Offset Y (deg)", limelight::getTargetOffsetY);
    limel.addNumber("Distance to Target (m)", limelight::getDistanceToTarget);
    limel.addNumber("Horizontal Dist (m)", limelight::getHorizontalDistanceToTarget);
    limel.addString("Status", limelight::getDebugString);
    limel.addNumber("Desired Tag ID", limelight::getDesiredTagID);

    // Shooter layout
    ShuffleboardLayout sh = tab.getLayout("PID Shooter Testing", BuiltInLayouts.kList)
        .withSize(3, 7)
        .withPosition(3, 0);

    sh.addNumber("Flywheel RPM", shooter::getFlywheelRPM);
    sh.addNumber("Flywheel Target RPM", shooter::getDashboardTargetRPM);
    sh.addBoolean("At Target RPM", () -> shooter.isAtTargetRPM(shooter.getDashboardTargetRPM(), frc.robot.Constants.ShooterConstants.kShooterRpmTolerance));
    sh.addNumber("RPM Error", () -> shooter.getFlywheelRPM() - shooter.getDashboardTargetRPM());
    sh.addNumber("Last Flywheel Speed", () -> edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(Constants.LimelightConstants.kShooterLastFlexSpeedKey, 0.0));

    // Intake / Belt layout
    ShuffleboardLayout feed = tab.getLayout("Intake/Belt", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(6, 0);

    feed.addNumber("Intake Last Speed", () -> edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(Constants.LimelightConstants.kIntakeLastSpeedKey, 0.0));
    feed.addNumber("Belt Last Speed", () -> edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(Constants.LimelightConstants.kBeltLastSpeedKey, 0.0));

    // Drive / Odometry
    ShuffleboardLayout driveL = tab.getLayout("Drive", BuiltInLayouts.kList)
        .withSize(3, 4)
        .withPosition(0, 7);

    driveL.addNumber("Pose X (m)", () -> drive.getPose().getX());
    driveL.addNumber("Pose Y (m)", () -> drive.getPose().getY());
    driveL.addNumber("Heading (deg)", () -> drive.getHeading());
    driveL.addNumber("Turn Rate (deg/s)", () -> drive.getTurnRate());

    // Tuning and status layout
    ShuffleboardLayout misc = tab.getLayout("Tuning/Status", BuiltInLayouts.kList)
        .withSize(4, 4)
        .withPosition(3, 7);

    misc.addNumber("PID Shooter Testing/Target RPM", shooter::getDashboardTargetRPM);
    misc.addNumber("PID Shooter Testing/PID P", () -> edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("PID Shooter Testing/PID P Gain", Constants.ShooterConstants.kFlywheelP));
    misc.addNumber("PID Shooter Testing/PID I", () -> edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("PID Shooter Testing/PID I Gain", Constants.ShooterConstants.kFlywheelI));
    misc.addNumber("PID Shooter Testing/PID D", () -> edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("PID Shooter Testing/PID D Gain", Constants.ShooterConstants.kFlywheelD));
    misc.addNumber("Pipeline Latency (ms)", () -> edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(Constants.LimelightConstants.kDashboardPipelineLatencyKey, 0.0));
    misc.addString("Vision Status", () -> edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString(Constants.LimelightConstants.kDashboardStatusKey, ""));

    // Make it easier for the dashboard to auto-layout widgets (elastic feel)
    // Widgets are added as list layouts which shuffleboard can reflow when resized.
  }
}
