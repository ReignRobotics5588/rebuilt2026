package frc.robot.sim;

import frc.robot.subsystems.sim.DriveSimulation;

/**
 * Container for all robot simulations.
 * Manages the update of all subsystem simulations during simulation mode.
 */
public class RobotSimulation {
  
  private static RobotSimulation instance;
  
  private final DriveSimulation m_driveSimulation;
  private boolean m_isSimulationEnabled = false;
  
  private RobotSimulation() {
    m_driveSimulation = new DriveSimulation();
  }
  
  /**
   * Get the singleton instance of RobotSimulation
   */
  public static RobotSimulation getInstance() {
    if (instance == null) {
      instance = new RobotSimulation();
    }
    return instance;
  }
  
  /**
   * Initialize simulation (called when robot enters simulation mode)
   */
  public void init() {
    m_isSimulationEnabled = true;
  }
  
  /**
   * Update all simulations (called every robot cycle during simulation)
   * @param deltaTime time since last update in seconds
   */
  public void update(double deltaTime) {
    if (!m_isSimulationEnabled) {
      return;
    }
    
    // Update drive simulation with dummy voltages
    // In a real implementation, these would come from the motor controllers
    double[] driveVoltages = new double[4];
    double[] turnVoltages = new double[4];
    
    m_driveSimulation.update(driveVoltages, turnVoltages, deltaTime);
  }
  
  /**
   * Get the drive simulation
   */
  public DriveSimulation getDriveSimulation() {
    return m_driveSimulation;
  }
  
  /**
   * Check if simulation is enabled
   */
  public boolean isSimulationEnabled() {
    return m_isSimulationEnabled;
  }
}
