package frc.robot.subsystems.sim;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.SimulationConstants;

/**
 * Simulation of the swerve drive subsystem.
 * Simulates motor behavior, encoder positions, and robot movement.
 */
public class DriveSimulation {
  
  private final double[] m_driveVoltages = new double[4];
  private final double[] m_turnVoltages = new double[4];
  private final double[] m_driveEncoderPositions = new double[4];
  private final double[] m_driveEncoderVelocities = new double[4];
  private final double[] m_turnEncoderPositions = new double[4];
  private final double[] m_turnEncoderVelocities = new double[4];
  
  private Pose2d m_simRobotPose = new Pose2d();
  
  public DriveSimulation() {
    // Initialize all encoders to zero
    for (int i = 0; i < 4; i++) {
      m_driveEncoderPositions[i] = 0.0;
      m_driveEncoderVelocities[i] = 0.0;
      m_turnEncoderPositions[i] = 0.0;
      m_turnEncoderVelocities[i] = 0.0;
    }
  }
  
  /**
   * Update drive simulation with motor voltages
   * @param driveVoltages voltage for each drive motor (0-3)
   * @param turnVoltages voltage for each turn motor (0-3)
   * @param deltaTime time since last update in seconds
   */
  public void update(double[] driveVoltages, double[] turnVoltages, double deltaTime) {
    // Clamp voltages to battery voltage
    for (int i = 0; i < 4; i++) {
      m_driveVoltages[i] = clampVoltage(driveVoltages[i]);
      m_turnVoltages[i] = clampVoltage(turnVoltages[i]);
    }
    
    // Update encoder positions based on voltages and motor characteristics
    updateDriveEncoders(deltaTime);
    updateTurnEncoders(deltaTime);
    
    // Update battery simulation
    updateBatterySim();
  }
  
  /**
   * Update drive encoder positions and velocities
   */
  private void updateDriveEncoders(double deltaTime) {
    // Constants for motor dynamics
    final double kMaxVelocity = SimulationConstants.DriveSimConstants.kDriveMotorMaxRPM / 60.0;
    final double kTimeConstant = 0.02;  // Motor response time in seconds
    
    for (int i = 0; i < 4; i++) {
      // Convert voltage to target velocity (RPM)
      double targetVelocity = (m_driveVoltages[i] / 12.0) * kMaxVelocity;
      
      // First-order filter to simulate motor acceleration
      double acceleration = (targetVelocity - m_driveEncoderVelocities[i]) / kTimeConstant;
      m_driveEncoderVelocities[i] += acceleration * deltaTime;
      
      // Update position
      m_driveEncoderPositions[i] += m_driveEncoderVelocities[i] * deltaTime;
    }
  }
  
  /**
   * Update turn encoder positions and velocities
   */
  private void updateTurnEncoders(double deltaTime) {
    // Constants for motor dynamics
    final double kMaxVelocity = SimulationConstants.DriveSimConstants.kTurnMotorMaxRPM / 60.0;
    final double kTimeConstant = 0.02;
    
    for (int i = 0; i < 4; i++) {
      // Convert voltage to target velocity (RPM)
      double targetVelocity = (m_turnVoltages[i] / 12.0) * kMaxVelocity;
      
      // First-order filter to simulate motor acceleration
      double acceleration = (targetVelocity - m_turnEncoderVelocities[i]) / kTimeConstant;
      m_turnEncoderVelocities[i] += acceleration * deltaTime;
      
      // Update position
      m_turnEncoderPositions[i] += m_turnEncoderVelocities[i] * deltaTime;
      
      // Wrap turn position to 0-1 range (one revolution)
      m_turnEncoderPositions[i] = m_turnEncoderPositions[i] % 1.0;
    }
  }
  
  /**
   * Update battery simulation based on motor current draw
   */
  private void updateBatterySim() {
    // Battery simulation can be added if needed
    // For now, we'll keep it simple
  }
  
  /**
   * Get the current drive encoder position for a module
   * @param module module index (0-3)
   * @return position in encoder units
   */
  public double getDriveEncoderPosition(int module) {
    return m_driveEncoderPositions[module];
  }
  
  /**
   * Get the current drive encoder velocity for a module
   * @param module module index (0-3)
   * @return velocity in encoder units per second
   */
  public double getDriveEncoderVelocity(int module) {
    return m_driveEncoderVelocities[module];
  }
  
  /**
   * Get the current turn encoder position for a module
   * @param module module index (0-3)
   * @return position (0-1 representing one full rotation)
   */
  public double getTurnEncoderPosition(int module) {
    return m_turnEncoderPositions[module];
  }
  
  /**
   * Get the current turn encoder velocity for a module
   * @param module module index (0-3)
   * @return velocity in rotations per second
   */
  public double getTurnEncoderVelocity(int module) {
    return m_turnEncoderVelocities[module];
  }
  
  /**
   * Reset all encoders to zero
   */
  public void resetEncoders() {
    for (int i = 0; i < 4; i++) {
      m_driveEncoderPositions[i] = 0.0;
      m_driveEncoderVelocities[i] = 0.0;
      m_turnEncoderPositions[i] = 0.0;
      m_turnEncoderVelocities[i] = 0.0;
    }
  }
  
  /**
   * Get the simulated robot pose
   * @return current pose
   */
  public Pose2d getRobotPose() {
    return m_simRobotPose;
  }
  
  /**
   * Set the simulated robot pose
   * @param pose new pose
   */
  public void setRobotPose(Pose2d pose) {
    m_simRobotPose = pose;
  }
  
  /**
   * Clamp voltage to valid battery range
   */
  private double clampVoltage(double voltage) {
    return Math.max(-12.0, Math.min(12.0, voltage));
  }
}
