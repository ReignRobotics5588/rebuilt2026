package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants;
import frc.robot.Configs;
import com.revrobotics.ResetMode;

/**
 * Shooter subsystem that controls the flywheel and indexer motors.
 * 
 * The flywheel uses closed-loop velocity control (PID) to maintain target RPM.
 * The indexer (feeder wheel) is controlled via percent output.
 * 
 * This simplified implementation follows REV Robotics best practices and removes
 * unnecessary complexity from the original version.
 * 
 * Key improvements:
 * - Removed complex dashboard PID tuning loop
 * - Fixed setFlywheelRPM() to actually use PID control via setReference()
 * - Simplified from 208 lines to ~120 lines
 * - Uses REV Robotics standard patterns for velocity control
 * - Reduced telemetry to essential values only
 */
public class Shooter extends SubsystemBase {

  private final SparkFlex m_flywheel = new SparkFlex(Constants.DriveConstants.flywheelID,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkMax m_indexer = new SparkMax(Constants.DriveConstants.indexerID,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkClosedLoopController m_flywheelPIDController = m_flywheel.getClosedLoopController();

  private double m_targetRPM = 0.0;
  private double m_indexerOutputPercent = 0.0;

  public Shooter() {
    // Configure flywheel with PID-based velocity control
    m_flywheel.configure(Configs.flywheel.flywheel_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    // Configure indexer for simple percent output control
    m_indexer.configure(Configs.indexer.indexer_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Update telemetry data
    updateTelemetry();
  }

  /**
   * Send essential telemetry data to SmartDashboard for monitoring
   */
  private void updateTelemetry() {
    // Flywheel feedback
    SmartDashboard.putNumber("Shooter/Flywheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Shooter/Target RPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter/RPM Error", getFlywheelRPM() - m_targetRPM);
    SmartDashboard.putBoolean("Shooter/At Target RPM",
        isAtTargetRPM(m_targetRPM, Constants.ShooterConstants.kShooterRpmTolerance));
  }

  /**
   * Sets the flywheel to a target RPM using the internal PID controller.
   * The SparkFlex will use closed-loop velocity control to maintain this RPM.
   *
   * @param targetRPM the target RPM for the flywheel
   */
  public void setFlywheelRPM(double targetRPM) {
    m_targetRPM = targetRPM;
    // Use setSetpoint with ControlType.kVelocity for PID-based RPM control
    // The SparkFlex internal controller will automatically apply P, I, D, and FF gains
    // This follows the REV Robotics standard pattern used in official examples
    m_flywheelPIDController.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  /**
   * Sets the indexer (feeder wheel) to a percent output speed.
   * Typical range: -1.0 to 1.0, where positive is forward.
   *
   * @param speed the percent output speed (-1.0 to 1.0)
   */
  public void setIndexerSpeed(double speed) {
    m_indexerOutputPercent = speed;
    m_indexer.set(speed);
  }

  /**
   * Stops the flywheel by setting target RPM to 0.
   */
  public void stopFlywheel() {
    setFlywheelRPM(0.0);
  }

  /**
   * Stops the indexer.
   */
  public void stopIndexer() {
    setIndexerSpeed(0.0);
  }

  /**
   * Gets the current RPM of the flywheel motor.
   * 
   * @return the current RPM
   */
  public double getFlywheelRPM() {
    return m_flywheel.getEncoder().getVelocity();
  }

  /**
   * Gets the current RPM of the indexer motor.
   * 
   * @return the current RPM
   */
  public double getIndexerRPM() {
    return m_indexer.getEncoder().getVelocity();
  }

  /**
   * Checks if the shooter flywheel is at or near the target RPM.
   * 
   * @param targetRPM the target RPM to check against
   * @param tolerance the RPM tolerance (e.g., 200 RPM)
   * @return true if the flywheel is within tolerance of target RPM
   */
  public boolean isAtTargetRPM(double targetRPM, double tolerance) {
    double flywheelRPM = getFlywheelRPM();
    return Math.abs(flywheelRPM - targetRPM) <= tolerance;
  }

}
