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
import frc.robot.commands.ShooterPIDTestCommand;
import frc.robot.commands.LimelightAlignCommand;
import frc.robot.commands.IntakeThenShootAutoCommand;
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
        m_shooter.setFlywheelSpeed(0);
        m_shooter.setIndexerSpeed(0);
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
    // Default: do nothing
    m_autoChooser.setDefaultOption("Do Nothing (default)",
        Commands.none());

    // Drive back 1.5m only
    m_autoChooser.addOption("Drive Back 1.5m", new DriveDistanceCommand(m_robotDrive, -1.5, 0.3));

    // Turn left 45 degrees only
    m_autoChooser.addOption("Turn Left 45°", new TurnToAngleCommand(m_robotDrive, 45.0));

    // Intake and shoot: Run intake, then shoot once flywheel reaches 1000 RPM
    m_autoChooser.addOption("Intake -> Shoot at 1000 RPM",
        new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt, Constants.ShooterConstants.kShooterAutoRPM));

    // Full sequence: drive back, turn left, limelight align, shoot
    m_autoChooser.addOption("DriveBack->TurnLeft->Aim->Shoot (45°)",
        AutoCommandFactory.driveBackTurnAimShoot(m_robotDrive, m_limelight, m_shooter, m_belt, 45.0));

    SmartDashboard.putData("Autonomous Mode", m_autoChooser);
  }

  /**
   * Periodic method to update dashboard-driven values.
   * This runs every robot cycle (20ms) and keeps Limelight target and Shooter PID synced with dashboard.
   * Called from Robot.robotPeriodic() before CommandScheduler.
   */
  public void periodic() {
    // ===== LIMELIGHT: Update desired tag ID =====
    // Check dashboard first for manual override
    int dashboardTagID = (int) SmartDashboard.getNumber(Constants.LimelightConstants.kDashboardTargetTagIdKey, -1);
    
    // If dashboard has been changed to something other than -1 (any tag), use it as override
    // Otherwise, use alliance-based default
    if (dashboardTagID != -1) {
      // Dashboard has a specific tag set - use it
      m_limelight.setDesiredTagID(dashboardTagID);
    } else {
      // Dashboard is at default (-1 = any tag), so use alliance-based default
      String allianceName = DriverStation.getAlliance().map(Enum::name).orElse("Invalid");
      if ("RED".equalsIgnoreCase(allianceName)) {
        m_limelight.setDesiredTagID(Constants.LimelightConstants.kRedAllianceTargetTagID);
      } else if ("BLUE".equalsIgnoreCase(allianceName)) {
        m_limelight.setDesiredTagID(Constants.LimelightConstants.kBlueAllianceTargetTagID);
      } else {
        // Unknown alliance - allow any tag
        m_limelight.setDesiredTagID(-1);
      }
    }

    // ===== SHOOTER: Update PID gains and target RPM =====
    double shooterP = SmartDashboard.getNumber("PID Shooter Testing/PID P Gain", Constants.ShooterConstants.kFlywheelP);
    double shooterI = SmartDashboard.getNumber("PID Shooter Testing/PID I Gain", Constants.ShooterConstants.kFlywheelI);
    double shooterD = SmartDashboard.getNumber("PID Shooter Testing/PID D Gain", Constants.ShooterConstants.kFlywheelD);
    double shooterTargetRPM = SmartDashboard.getNumber("PID Shooter Testing/Target RPM", Constants.ShooterConstants.kShooterTargetRPM);
    
    // Apply any PID changes and update target RPM
    m_shooter.updatePIDFromDashboard(shooterP, shooterI, shooterD, shooterTargetRPM);
    
    // ===== SHOOTER TELEMETRY: Display current performance and PID gains =====
    // Display current and target RPM for monitoring (updated every cycle via getters)
    SmartDashboard.putNumber("Shooter/Current RPM", m_shooter.getFlywheelRPM());
    SmartDashboard.putNumber("Shooter/Target RPM", m_shooter.getTargetRPM());
    
    // Display editable PID gains (these are the live values from dashboard being applied)
    SmartDashboard.putNumber("Shooter/P Gain", shooterP);
    SmartDashboard.putNumber("Shooter/I Gain", shooterI);
    SmartDashboard.putNumber("Shooter/D Gain", shooterD);
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
    
    // Spit out command: Run intake in reverse at 0.7 power
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .toggleOnTrue(new RunCommand(() -> m_intake.setSpeed(0.7), m_intake));
    
    // Shooter PID Test: Use right bumper to test PID tuning
    // Reads dashboard target RPM and uses PID-controlled setFlywheelRPM()
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .toggleOnTrue(new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt, 3200));
    
/*     // Shooter + Belt: Shooter ramps to speed first, then belt engages
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .toggleOnTrue(new ShooterBeltCommand(m_shooter, m_belt));
*/

    // Shoot at wall
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .toggleOnTrue(new IntakeThenShootAutoCommand(m_intake, m_shooter, m_belt, Constants.ShooterConstants.kShooterAutoRPM));


    
    // Limelight vision alignment: Align to detected April tag
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new LimelightAlignCommand(m_robotDrive, m_limelight));
    
    // Old individual button bindings (kept for reference - remove if no longer needed)
    // new JoystickButton(m_driverController, XboxController.Button.kB.value).toggleOnTrue(new RunCommand(() -> m_intake.setSpeed(Constants.BeltConstants.kBeltSpeed), m_intake));
    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).toggleOnTrue(new RunCommand(() -> m_shooter.setIndexerSpeed(Constants.ShooterConstants.kShooterSpeed), m_shooter));
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
