package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.FeedbackSensor;
import frc.robot.Constants;

import frc.robot.Configs;
import com.revrobotics.ResetMode;

public class Shooter extends SubsystemBase {

  private final SparkFlex m_flywheel = new SparkFlex(Constants.DriveConstants.flywheelID,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkMax m_indexer = new SparkMax(Constants.DriveConstants.indexerID,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkClosedLoopController m_flywheelPIDController = m_flywheel.getClosedLoopController();

  private double m_targetRPM = 0.0;
  private double m_flywheelOutputPercent = 0.0;
  private double m_indexerOutputPercent = 0.0;
  private double m_lastFlywheelSpeed = 0.0;
  private double m_lastIndexerSpeed = 0.0;

  // PID tuning values from dashboard
  private double m_dashboardP = Constants.ShooterConstants.kFlywheelP;
  private double m_dashboardI = Constants.ShooterConstants.kFlywheelI;
  private double m_dashboardD = Constants.ShooterConstants.kFlywheelD;
  private double m_dashboardFF = Constants.ShooterConstants.kFlywheelFF;
  private double m_dashboardTargetRPM = Constants.ShooterConstants.kShooterTargetRPM;

  public Shooter() {
    m_flywheel.configure(Configs.flywheel.flywheel_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_indexer.configure(Configs.indexer.indexer_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize dashboard with current PID values
    initializeDashboard();
  }

  /**
   * Initialize dashboard with default PID values and RPM target
   */
  private void initializeDashboard() {
    SmartDashboard.putNumber("Shooter/PID P Gain", Constants.ShooterConstants.kFlywheelP);
    SmartDashboard.putNumber("Shooter/PID I Gain", Constants.ShooterConstants.kFlywheelI);
    SmartDashboard.putNumber("Shooter/PID D Gain", Constants.ShooterConstants.kFlywheelD);
    SmartDashboard.putNumber("Shooter/PID FF Gain", Constants.ShooterConstants.kFlywheelFF);
    SmartDashboard.putNumber("Shooter/Target RPM", Constants.ShooterConstants.kShooterTargetRPM);
    SmartDashboard.putNumber("Shooter/RPM Tolerance", Constants.ShooterConstants.kShooterRpmTolerance);
  }

  @Override
  public void periodic() {
    // Check for PID changes from dashboard
    checkAndApplyPIDChanges();

    // Update target RPM from dashboard
    m_dashboardTargetRPM = SmartDashboard.getNumber("Shooter/Target RPM", m_dashboardTargetRPM);

    // Update telemetry data for Glass
    updateTelemetry();
  }

  /**
   * Check if PID values changed on dashboard and apply them to the motor
   * controller
   */
  private void checkAndApplyPIDChanges() {
    double newP = SmartDashboard.getNumber("Shooter/PID P Gain", m_dashboardP);
    double newI = SmartDashboard.getNumber("Shooter/PID I Gain", m_dashboardI);
    double newD = SmartDashboard.getNumber("Shooter/PID D Gain", m_dashboardD);
    double newFF = SmartDashboard.getNumber("Shooter/PID FF Gain", m_dashboardFF);

    // Check if any value changed
    if (newP != m_dashboardP || newI != m_dashboardI || newD != m_dashboardD || newFF != m_dashboardFF) {
      m_dashboardP = newP;
      m_dashboardI = newI;
      m_dashboardD = newD;
      m_dashboardFF = newFF;

      // Create new config with updated PID values
      SparkFlexConfig updatedConfig = new SparkFlexConfig();
      updatedConfig
          .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
          .smartCurrentLimit(40);

      updatedConfig.encoder
          .velocityConversionFactor(1.0);

      updatedConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(m_dashboardP, m_dashboardI, m_dashboardD)
          .outputRange(-1.0, 1.0);

      // Apply new configuration
      m_flywheel.configure(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      SmartDashboard.putString("Shooter/Status", String.format(
          "[Shooter] PID Updated: P=%.6f, I=%.6f, D=%.6f, FF=%.6f",
          m_dashboardP, m_dashboardI, m_dashboardD, m_dashboardFF));
    }
  }

  /**
   * Send telemetry data to SmartDashboard/Glass for PID shooter monitoring
   */
  private void updateTelemetry() {
    // Flywheel (PID-controlled)
    SmartDashboard.putNumber("Shooter/Flywheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Shooter/Flywheel Target RPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter/Flywheel Output %", m_flywheelOutputPercent);
    SmartDashboard.putBoolean("Shooter/At Target RPM",
        isAtTargetRPM(m_targetRPM, Constants.ShooterConstants.kShooterRpmTolerance));
    SmartDashboard.putNumber("Shooter/RPM Error",
        getFlywheelRPM() - m_targetRPM);

    // Indexer (feeder wheel)
    SmartDashboard.putNumber("Shooter/Indexer RPM", getIndexerRPM());
    SmartDashboard.putNumber("Shooter/Indexer Output %", m_indexerOutputPercent);
    // Last commanded speeds
    SmartDashboard.putNumber(Constants.LimelightConstants.kShooterLastFlexSpeedKey, m_lastFlywheelSpeed);
    SmartDashboard.putNumber(Constants.LimelightConstants.kShooterLastMaxSpeedKey, m_lastIndexerSpeed);

    // Dashboard tuning values
    SmartDashboard.putNumber("Shooter/Dashboard Target RPM", m_dashboardTargetRPM);

    // Current PID values (read-only display in telemetry section)
    SmartDashboard.putNumber("Shooter/Active P Gain", m_dashboardP);
    SmartDashboard.putNumber("Shooter/Active I Gain", m_dashboardI);
    SmartDashboard.putNumber("Shooter/Active D Gain", m_dashboardD);
    SmartDashboard.putNumber("Shooter/Active FF Gain", m_dashboardFF);
    SmartDashboard.putNumber("Shooter/RPM Tolerance", Constants.ShooterConstants.kShooterRpmTolerance);
  }

  public void setFlywheelSpeed(double speed) {
    m_lastFlywheelSpeed = speed;
    m_flywheelOutputPercent = speed;
    m_flywheel.set(speed);
  }

  public void setIndexerSpeed(double speed) {
    m_lastIndexerSpeed = speed;
    m_indexerOutputPercent = speed;
    m_indexer.set(speed);
  }

  /**
   * Sets the SparkFlex shooter to a target RPM using the internal PID controller.
   * This ensures the flywheel maintains consistent speed.
   *
   * @param targetRPM the target RPM for the flywheel
   */
  public void setFlywheelRPM(double targetRPM) {
    m_targetRPM = targetRPM;
    // Use the setSetpoint method with kVelocity control type
    m_flywheelPIDController.setSetpoint(targetRPM, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
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
   * Gets the target RPM from the dashboard (tunable during testing).
   * 
   * @return the dashboard target RPM
   */
  public double getDashboardTargetRPM() {
    return m_dashboardTargetRPM;
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
