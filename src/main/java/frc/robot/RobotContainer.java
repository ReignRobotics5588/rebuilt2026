package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.IntakeBeltCommand;
import frc.robot.commands.IntakeThenShootAutoCommand;
import frc.robot.commands.LimelightAlignCommand;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.io.BeltIOYAMS;
import frc.robot.subsystems.io.DriveIOYAMS;
import frc.robot.subsystems.io.IntakeIOYAMS;
import frc.robot.subsystems.io.ShooterIOYAMS;

public class RobotContainer {

  public static final DriveSubsystem m_robotDrive;
  public static final Intake m_intake;
  public static final Shooter m_shooter;
  public static final Belt m_belt;
  public static final Limelight m_limelight = new Limelight();

  static {
    m_intake = new Intake(new IntakeIOYAMS());
    m_shooter = new Shooter(new ShooterIOYAMS());
    m_belt = new Belt(new BeltIOYAMS());
    NamedCommands.registerCommand(AutoConstants.kIntakeOn,
        Commands.runOnce(() -> m_intake.setSpeed(IntakeConstants.kIntakeSpeed), m_intake));
    NamedCommands.registerCommand(AutoConstants.kIntakeOff,
        Commands.runOnce(() -> m_intake.setSpeed(0), m_intake));
    NamedCommands.registerCommand(AutoConstants.kBeltOn,
        Commands.runOnce(() -> m_belt.setSpeed(Constants.BeltConstants.kBeltSpeed), m_belt));
    NamedCommands.registerCommand(AutoConstants.kBeltOff,
        Commands.runOnce(() -> m_belt.setSpeed(0), m_belt));
    NamedCommands.registerCommand(AutoConstants.kShootAuto,
        new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt, ShooterConstants.kShooterAutoRPM)
            .withTimeout(AutoConstants.kShootTimeoutSeconds));
    // DriveSubsystem must exist before AutoBuilder-dependent and drive commands
    m_robotDrive = new DriveSubsystem(new DriveIOYAMS());
    NamedCommands.registerCommand(AutoConstants.kAimAndShoot,
        new AimAndShootCommand(m_robotDrive, m_shooter, m_intake, m_belt)
            .finishAfterFeeding(AutoConstants.kShootFeedSeconds)
            .withTimeout(AutoConstants.kShootTimeoutSeconds));
    NamedCommands.registerCommand(AutoConstants.kLimelightAlign,
        new LimelightAlignCommand(m_robotDrive, m_limelight));
  }

  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);
  private final DriverInput m_driverInput = new DriverInput(m_driverController);

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    configureButtonBindings();
    configureSysIdBindings();
    configureDefaultCommands();
    DriverStation.silenceJoystickConnectionWarning(true);

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Mode", m_autoChooser);
  }

  private void configureDefaultCommands() {
    // Skipped in auto so the IntakeOn/IntakeOff named commands aren't undone
    m_intake.setDefaultCommand(
        new RunCommand(() -> {
          if (!DriverStation.isAutonomous()) {
            m_intake.setSpeed(0);
          }
        }, m_intake));

    m_shooter.setDefaultCommand(
        new RunCommand(() -> {
          m_shooter.setFlywheelSpeed(0);
          m_shooter.setIndexerSpeed(0);
        }, m_shooter));

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                m_driverInput.forward(),
                m_driverInput.left(),
                m_driverInput.rotation(),
                true),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    Trigger notTest = new Trigger(DriverStation::isTest).negate();

    // ---- Driver ----
    m_driverController.x()
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    m_driverController.b()
        .whileTrue(new LimelightAlignCommand(m_robotDrive, m_limelight));

    // Hold to aim at the hub while driving; shoots at the distance-based RPM
    m_driverController.rightTrigger()
        .whileTrue(new AimAndShootCommand(m_robotDrive, m_shooter, m_intake, m_belt,
            m_driverInput::forward, m_driverInput::left));

    // Re-zero field-relative driving: point the robot away from the driver first
    m_driverController.start()
        .onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));

    // ---- Mechanisms: on both controllers until the operator role is settled ----
    for (CommandXboxController controller : new CommandXboxController[] {m_driverController, m_operatorController}) {
      controller.a().and(notTest)
          .toggleOnTrue(new IntakeBeltCommand(m_intake, m_belt));

      // Spit out ("puke")
      controller.leftBumper().and(notTest)
          .toggleOnTrue(Commands.startEnd(
              () -> m_intake.setSpeed(IntakeConstants.kEjectSpeed),
              () -> m_intake.setSpeed(0),
              m_intake));

      // Close shot
      controller.rightBumper().and(notTest)
          .toggleOnTrue(new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt,
              ShooterConstants.kShooterCloseRPM));

      // Shoot at wall
      controller.y().and(notTest)
          .toggleOnTrue(new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt,
              ShooterConstants.kShooterAutoRPM));
    }

    // Operator: hold to pre-spin the flywheel for the current hub distance
    m_operatorController.x().and(notTest)
        .whileTrue(Commands.run(() -> m_shooter.setFlywheelRPM(Shooter.getRPMForDistance(
            FieldConstants.getAllianceHubCenter().getDistance(m_robotDrive.getPose().getTranslation()))),
            m_shooter));
  }

  /**
   * SysId characterization, Test mode only, operator controller. Hold left bumper
   * for the flywheel, otherwise the drive. POV up/down = quasistatic fwd/rev,
   * POV right/left = dynamic fwd/rev. Hold until the routine has room/time to finish.
   */
  private void configureSysIdBindings() {
    Trigger test = new Trigger(DriverStation::isTest);
    Trigger flywheel = test.and(m_operatorController.leftBumper());
    Trigger drive = test.and(m_operatorController.leftBumper().negate());

    drive.and(m_operatorController.povUp()).whileTrue(m_robotDrive.sysIdQuasistatic(Direction.kForward));
    drive.and(m_operatorController.povDown()).whileTrue(m_robotDrive.sysIdQuasistatic(Direction.kReverse));
    drive.and(m_operatorController.povRight()).whileTrue(m_robotDrive.sysIdDynamic(Direction.kForward));
    drive.and(m_operatorController.povLeft()).whileTrue(m_robotDrive.sysIdDynamic(Direction.kReverse));

    flywheel.and(m_operatorController.povUp()).whileTrue(m_shooter.sysIdQuasistatic(Direction.kForward));
    flywheel.and(m_operatorController.povDown()).whileTrue(m_shooter.sysIdQuasistatic(Direction.kReverse));
    flywheel.and(m_operatorController.povRight()).whileTrue(m_shooter.sysIdDynamic(Direction.kForward));
    flywheel.and(m_operatorController.povLeft()).whileTrue(m_shooter.sysIdDynamic(Direction.kReverse));
  }

  public void periodic() {
    // Runs before the scheduler, so drive commands see this loop's sticks
    m_driverInput.update();

    double yawRate = m_robotDrive.getRobotRelativeSpeed().omegaRadiansPerSecond;
    m_limelight.setRobotOrientation(m_robotDrive.getPose().getRotation(), yawRate);

    m_limelight.getLatestPoseEstimate().ifPresent(pe -> {
      // MegaTag2 is unreliable while spinning faster than ~720 deg/s
      if (pe.hasData && pe.tagCount >= 1 && Math.abs(yawRate) < 4 * Math.PI) {
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
    SmartDashboard.putNumber("Shooter/Hub Distance (m)",
        FieldConstants.getAllianceHubCenter().getDistance(m_robotDrive.getPose().getTranslation()));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
