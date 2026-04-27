package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.BeltIOSparkMax;
import frc.robot.subsystems.BeltIOSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroIOPigeon2;
import frc.robot.subsystems.GyroIOSim;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeIOSparkMax;
import frc.robot.subsystems.IntakeIOSim;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ModuleIOSparkMax;
import frc.robot.subsystems.ModuleIOSim;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterIOSparkFlex;
import frc.robot.subsystems.ShooterIOSim;
import frc.robot.subsystems.VisionIOLimelight;
import frc.robot.subsystems.VisionIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.IntakeBeltCommand;
import frc.robot.commands.LimelightAlignCommand;
import frc.robot.commands.IntakeThenShootAutoCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // Subsystems — wired with real hardware or simulation IO based on current mode
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem(
      Constants.currentMode == Constants.Mode.REAL ? new GyroIOPigeon2() : new GyroIOSim(),
      Constants.currentMode == Constants.Mode.REAL
          ? new ModuleIOSparkMax(DriveConstants.kFrontLeftDrivingCanId,
              DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset)
          : new ModuleIOSim(),
      Constants.currentMode == Constants.Mode.REAL
          ? new ModuleIOSparkMax(DriveConstants.kFrontRightDrivingCanId,
              DriveConstants.kFrontRightTurningCanId,
              DriveConstants.kFrontRightChassisAngularOffset)
          : new ModuleIOSim(),
      Constants.currentMode == Constants.Mode.REAL
          ? new ModuleIOSparkMax(DriveConstants.kRearLeftDrivingCanId,
              DriveConstants.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset)
          : new ModuleIOSim(),
      Constants.currentMode == Constants.Mode.REAL
          ? new ModuleIOSparkMax(DriveConstants.kRearRightDrivingCanId,
              DriveConstants.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset)
          : new ModuleIOSim());

  public static final Intake m_intake = new Intake(
      Constants.currentMode == Constants.Mode.REAL ? new IntakeIOSparkMax() : new IntakeIOSim());

  public static final Shooter m_shooter = new Shooter(
      Constants.currentMode == Constants.Mode.REAL ? new ShooterIOSparkFlex() : new ShooterIOSim());

  public static final Belt m_belt = new Belt(
      Constants.currentMode == Constants.Mode.REAL ? new BeltIOSparkMax() : new BeltIOSim());

  public static final Limelight m_limelight = new Limelight(
      Constants.currentMode == Constants.Mode.REAL ? new VisionIOLimelight() : new VisionIOSim());

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setSpeed(0), m_intake));

    m_shooter.setDefaultCommand(new RunCommand(() -> {
      m_shooter.setFlywheelSpeed(0);
      m_shooter.setIndexerSpeed(0);
    }, m_shooter));

    m_robotDrive.setDefaultCommand(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
            true),
        m_robotDrive));

    m_autoChooser.setDefaultOption("Do Nothing (default)", Commands.none());
    m_autoChooser.addOption("Drive Back 1.5m", new DriveDistanceCommand(m_robotDrive, -1.5, 0.3));
    m_autoChooser.addOption("Turn Left 45°", new TurnToAngleCommand(m_robotDrive, 45.0));
    m_autoChooser.addOption("Intake -> Shoot at 1000 RPM",
        new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt,
            Constants.ShooterConstants.kShooterAutoRPM));
    m_autoChooser.addOption("DriveBack->TurnLeft->Aim->Shoot (45°)",
        AutoCommandFactory.driveBackTurnAimShoot(m_robotDrive, m_limelight, m_shooter, m_belt, 45.0));

    SmartDashboard.putData("Autonomous Mode", m_autoChooser);
  }

  public void periodic() {
    // Limelight desired tag ID (alliance-based default or dashboard override)
    int dashboardTagID = (int) SmartDashboard.getNumber(
        Constants.LimelightConstants.kDashboardTargetTagIdKey, -1);
    if (dashboardTagID != -1) {
      m_limelight.setDesiredTagID(dashboardTagID);
    } else {
      String alliance = DriverStation.getAlliance().map(Enum::name).orElse("Invalid");
      if ("RED".equalsIgnoreCase(alliance)) {
        m_limelight.setDesiredTagID(Constants.LimelightConstants.kRedAllianceTargetTagID);
      } else if ("BLUE".equalsIgnoreCase(alliance)) {
        m_limelight.setDesiredTagID(Constants.LimelightConstants.kBlueAllianceTargetTagID);
      } else {
        m_limelight.setDesiredTagID(-1);
      }
    }

    // Shooter PID tuning from dashboard
    double shooterP = SmartDashboard.getNumber("PID Shooter Testing/PID P Gain",
        Constants.ShooterConstants.kFlywheelP);
    double shooterI = SmartDashboard.getNumber("PID Shooter Testing/PID I Gain",
        Constants.ShooterConstants.kFlywheelI);
    double shooterD = SmartDashboard.getNumber("PID Shooter Testing/PID D Gain",
        Constants.ShooterConstants.kFlywheelD);
    double shooterRPM = SmartDashboard.getNumber("PID Shooter Testing/Target RPM",
        Constants.ShooterConstants.kShooterTargetRPM);
    m_shooter.updatePIDFromDashboard(shooterP, shooterI, shooterD, shooterRPM);

    SmartDashboard.putNumber("Shooter/P Gain", shooterP);
    SmartDashboard.putNumber("Shooter/I Gain", shooterI);
    SmartDashboard.putNumber("Shooter/D Gain", shooterD);
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
        .toggleOnTrue(new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt,
            Constants.ShooterConstants.kShooterAutoRPM));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new LimelightAlignCommand(m_robotDrive, m_limelight));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
