package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeBeltCommand;
import frc.robot.commands.IntakeThenShootAutoCommand;
import frc.robot.commands.LimelightAlignCommand;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final Intake m_intake = new Intake();
  public static final Shooter m_shooter = new Shooter();
  public static final Belt m_belt = new Belt();
  public static final Limelight m_limelight = new Limelight();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    registerNamedCommands();
    configureButtonBindings();
    configureDefaultCommands();
    DriverStation.silenceJoystickConnectionWarning(true);

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Mode", m_autoChooser);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("IntakeOn",
        Commands.runOnce(() -> m_intake.setSpeed(Constants.IntakeConstants.kIntakeSpeed), m_intake));
    NamedCommands.registerCommand("IntakeOff",
        Commands.runOnce(() -> m_intake.setSpeed(0), m_intake));
    NamedCommands.registerCommand("BeltOn",
        Commands.runOnce(() -> m_belt.setSpeed(Constants.BeltConstants.kBeltSpeed), m_belt));
    NamedCommands.registerCommand("BeltOff",
        Commands.runOnce(() -> m_belt.setSpeed(0), m_belt));
    NamedCommands.registerCommand("ShootAuto",
        new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt, ShooterConstants.kShooterAutoRPM));
    NamedCommands.registerCommand("LimelightAlign",
        new LimelightAlignCommand(m_robotDrive, m_limelight));
  }

  private void configureDefaultCommands() {
    m_intake.setDefaultCommand(
        new RunCommand(() -> m_intake.setSpeed(0), m_intake));

    m_shooter.setDefaultCommand(
        new RunCommand(() -> {
          m_shooter.setFlywheelSpeed(0);
          m_shooter.setIndexerSpeed(0);
        }, m_shooter));

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .toggleOnTrue(new IntakeBeltCommand(m_intake, m_belt));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .toggleOnTrue(new RunCommand(() -> m_intake.setSpeed(0.7), m_intake));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .toggleOnTrue(new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt, 3200));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .toggleOnTrue(new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt, ShooterConstants.kShooterAutoRPM));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new LimelightAlignCommand(m_robotDrive, m_limelight));
  }

  public void periodic() {
    m_limelight.getLatestPoseEstimate().ifPresent(pe -> {
      if (pe.hasData && pe.tagCount >= 1) {
        double distScale = (pe.avgTagDist * pe.avgTagDist) / pe.tagCount;
        m_robotDrive.addVisionMeasurement(
            pe.pose.toPose2d(),
            pe.timestampSeconds,
            VecBuilder.fill(0.1 * distScale, 0.1 * distScale, 9999999));
      }
    });

    int dashboardTagID = (int) SmartDashboard.getNumber(Constants.LimelightConstants.kDashboardTargetTagIdKey, -1);
    if (dashboardTagID != -1) {
      m_limelight.setDesiredTagID(dashboardTagID);
    } else {
      String allianceName = DriverStation.getAlliance().map(Enum::name).orElse("Invalid");
      if ("RED".equalsIgnoreCase(allianceName)) {
        m_limelight.setDesiredTagID(Constants.LimelightConstants.kRedAllianceTargetTagID);
      } else if ("BLUE".equalsIgnoreCase(allianceName)) {
        m_limelight.setDesiredTagID(Constants.LimelightConstants.kBlueAllianceTargetTagID);
      } else {
        m_limelight.setDesiredTagID(-1);
      }
    }

    SmartDashboard.putNumber("Shooter/Current RPM", m_shooter.getFlywheelRPM());
    SmartDashboard.putNumber("Shooter/Target RPM", m_shooter.getTargetRPM());
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
