package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.IntakeBeltCommand;
import frc.robot.commands.ShooterBeltCommand;

/**
 * Factory for creating autonomous commands using PathPlanner paths.
 * 
 * This class registers named commands that can be used as event markers in PathPlanner paths.
 * It also provides methods to load and execute saved paths.
 */
public class PathPlannerAutoFactory {
  
  private static boolean m_commandsRegistered = false;

  /**
   * Initialize PathPlanner and register all named commands.
   * Call this once during robot initialization.
   * 
   * @param drive the DriveSubsystem
   * @param shooter the Shooter subsystem
   * @param intake the Intake subsystem
   * @param belt the Belt subsystem
   */
  public static void registerCommands(
      DriveSubsystem drive,
      Shooter shooter,
      Intake intake,
      Belt belt) {
    
    if (m_commandsRegistered) {
      return;  // Already registered
    }
    
    // Register subsystem stop commands
    NamedCommands.registerCommand("Stop Shooter", 
        Commands.runOnce(() -> {
          shooter.setShooterFlexSpeed(0);
          shooter.setShooterMaxSpeed(0);
        }, shooter));
    
    NamedCommands.registerCommand("Stop Intake",
        Commands.runOnce(() -> intake.setSpeed(0), intake));
    
    NamedCommands.registerCommand("Stop Belt",
        Commands.runOnce(() -> belt.setSpeed(0), belt));
    
    // Register combined commands
    NamedCommands.registerCommand("Intake and Belt",
        new IntakeBeltCommand(intake, belt));
    
    NamedCommands.registerCommand("Shooter and Belt",
        new ShooterBeltCommand(shooter, belt));
    
    // Register shooter spin-up command
    NamedCommands.registerCommand("Spin Up Shooter",
        Commands.runOnce(() -> {
          shooter.setShooterFlexRPM(shooter.getDashboardTargetRPM());
          shooter.setShooterMaxSpeed(0.5);
        }, shooter));
    
    // Register intake command
    NamedCommands.registerCommand("Run Intake",
        Commands.runOnce(() -> intake.setSpeed(0.5), intake));
    
    // Register belt command
    NamedCommands.registerCommand("Run Belt",
        Commands.runOnce(() -> belt.setSpeed(0.33), belt));
    
    // Register wait command
    NamedCommands.registerCommand("Wait 0.5 Seconds",
        Commands.waitSeconds(0.5));
    
    NamedCommands.registerCommand("Wait 1 Second",
        Commands.waitSeconds(1.0));
    
    // Register debug/print command
    NamedCommands.registerCommand("Print Path Event",
        Commands.runOnce(() -> System.out.println("[PathPlanner] Event marker executed")));
    
    m_commandsRegistered = true;
    System.out.println("[PathPlannerAutoFactory] Commands registered successfully");
  }

  /**
   * Load and execute an autonomous routine from a saved PathPlanner path.
   * 
   * @param pathName the name of the path file (without .path extension)
   * @return the command to execute the path, or an empty command if not found
   */
  public static Command loadPath(String pathName) {
    try {
      return new PathPlannerAuto(pathName);
    } catch (Exception e) {
      System.err.println("[PathPlannerAutoFactory] Failed to load path: " + pathName);
      e.printStackTrace();
      return Commands.none();
    }
  }

  /**
   * Load multiple paths and execute them sequentially.
   * 
   * @param pathNames array of path names to execute in order
   * @return command that executes all paths sequentially
   */
  public static Command loadPathSequence(String... pathNames) {
    Command sequence = Commands.none();
    
    for (String pathName : pathNames) {
      sequence = sequence.andThen(loadPath(pathName));
    }
    
    return sequence;
  }

  /**
   * Load a path and combine it with a parallel command.
   * Useful for running intake/shooter while following a path.
   * 
   * @param pathName the path to follow
   * @param parallelCommand command to run in parallel with the path
   * @return combined command
   */
  public static Command loadPathWithParallel(String pathName, Command parallelCommand) {
    return loadPath(pathName).deadlineWith(parallelCommand);
  }

  /**
   * Get all available auto paths as an array.
   * Useful for populating a SendableChooser.
   * 
   * @return array of available path names
   */
  public static String[] getAvailablePaths() {
    // This would need to read from the pathplanner directory
    // For now, return an empty array
    return new String[]{};
  }

  /**
   * Example: Simple path following command
   */
  public static Command examplePath1() {
    return loadPath("Example Path 1");
  }

  /**
   * Example: Path with parallel intake
   */
  public static Command examplePath2WithIntake() {
    return loadPathWithParallel("Example Path 2", 
        Commands.runOnce(() -> RobotContainer.m_intake.setSpeed(0.5), RobotContainer.m_intake));
  }

  /**
   * Example: Multiple paths in sequence
   */
  public static Command examplePathSequence() {
    return loadPathSequence("Path 1", "Path 2", "Path 3");
  }
}
