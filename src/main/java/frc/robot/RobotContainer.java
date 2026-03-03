// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.IntakeBeltCommand;
import frc.robot.commands.ShooterBeltCommand;
import frc.robot.commands.LimelightAlignCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final Intake m_intake = new Intake();
  public static final Shooter m_shooter = new Shooter();
  public static final Belt m_belt = new Belt();
  public static final Limelight m_limelight = new Limelight();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // Autonomous chooser
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Default commands for subsystems
    m_intake.setDefaultCommand(
      new RunCommand(() -> m_intake.setSpeed(0), m_intake)
    );

    m_shooter.setDefaultCommand(
      new RunCommand(() -> {
        m_shooter.setShooterFlexSpeed(0);
        m_shooter.setShooterMaxSpeed(0);
      }, m_shooter)
    );

    // Configure default drive command (joystick control)
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    // Autonomous chooser: expose a few options on the dashboard
    // Default: original simple 2 second drive
    m_autoChooser.setDefaultOption("DriveForward 2s (default)",
        Commands.waitSeconds(2).deadlineFor(new RunCommand(() -> m_robotDrive.drive(-0.6, 0.0, 0.0, false), m_robotDrive)));

    // Drive back 1.5m only
    m_autoChooser.addOption("Drive Back 1.5m", new DriveDistanceCommand(m_robotDrive, -1.5, 0.3));

    // Turn left 45 degrees only
    m_autoChooser.addOption("Turn Left 45°", new TurnToAngleCommand(m_robotDrive, 45.0));

    // Full sequence: drive back, turn left, limelight align, shoot
    m_autoChooser.addOption("DriveBack->TurnLeft->Aim->Shoot (45°)",
        AutoCommandFactory.driveBackTurnAimShoot(m_robotDrive, m_limelight, m_shooter, m_belt, 45.0));

    SmartDashboard.putData("Autonomous Mode", m_autoChooser);
    // Build telemetry tab (elastic/list style) showing all important values
    TelemetryLayout.setup(m_robotDrive, m_limelight, m_shooter, m_belt, m_intake);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, XboxController.Button.kX.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    
    // Intake + Belt: Run together simultaneously
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .toggleOnTrue(new IntakeBeltCommand(m_intake, m_belt));
    
    // Shooter + Belt: Shooter ramps to speed first, then belt engages
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .toggleOnTrue(new ShooterBeltCommand(m_shooter, m_belt));
    
    // Limelight vision alignment: Align to detected April tag
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new LimelightAlignCommand(m_robotDrive, m_limelight));
    
    // Old individual button bindings (kept for reference - remove if no longer needed)
    // new JoystickButton(m_driverController, XboxController.Button.kB.value).toggleOnTrue(new RunCommand(() -> m_intake.setSpeed(Constants.BeltConstants.kBeltSpeed), m_intake));
    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).toggleOnTrue(new RunCommand(() -> m_shooter.setShooterMaxSpeed(Constants.ShooterConstants.kShooterSpeed), m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return the selected autonomous command from the dashboard chooser
    return m_autoChooser.getSelected();
  }
}
