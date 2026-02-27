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

    private final SparkFlex m_shooterFlex = new SparkFlex(Constants.DriveConstants.shooterFlexID, SparkLowLevel.MotorType.kBrushed);
    private final SparkMax m_shooterMax = new SparkMax(Constants.DriveConstants.shooterMaxID, SparkLowLevel.MotorType.kBrushed);
    private final SparkClosedLoopController m_flexPIDController = m_shooterFlex.getClosedLoopController();
    
    private double m_targetRPM = 0.0;
    private double m_flexOutputPercent = 0.0;
    private double m_maxOutputPercent = 0.0;
    
    // PID tuning values from dashboard
    private double m_dashboardP = Constants.ShooterConstants.kShooterFlexP;
    private double m_dashboardI = Constants.ShooterConstants.kShooterFlexI;
    private double m_dashboardD = Constants.ShooterConstants.kShooterFlexD;
    private double m_dashboardFF = Constants.ShooterConstants.kShooterFlexFF;
    private double m_dashboardTargetRPM = Constants.ShooterConstants.kShooterTargetRPM;

    public Shooter() {
        m_shooterFlex.configure(Configs.shooterFlex.shooterFlex_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterMax.configure(Configs.shooterMax.shooterMax_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Initialize dashboard with current PID values
        initializeDashboard();
    }

    /**
     * Initialize dashboard with default PID values and RPM target
     */
    private void initializeDashboard() {
        SmartDashboard.putNumber("Shooter/PID P Gain", Constants.ShooterConstants.kShooterFlexP);
        SmartDashboard.putNumber("Shooter/PID I Gain", Constants.ShooterConstants.kShooterFlexI);
        SmartDashboard.putNumber("Shooter/PID D Gain", Constants.ShooterConstants.kShooterFlexD);
        SmartDashboard.putNumber("Shooter/PID FF Gain", Constants.ShooterConstants.kShooterFlexFF);
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
     * Check if PID values changed on dashboard and apply them to the motor controller
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
            m_shooterFlex.configure(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            
            System.out.println(String.format(
                "[Shooter] PID Updated: P=%.6f, I=%.6f, D=%.6f, FF=%.6f",
                m_dashboardP, m_dashboardI, m_dashboardD, m_dashboardFF
            ));
        }
    }

    /**
     * Send telemetry data to SmartDashboard/Glass for PID shooter monitoring
     */
    private void updateTelemetry() {
        // Flex shooter (PID-controlled flywheel)
        SmartDashboard.putNumber("Shooter/Flex RPM", getFlexRPM());
        SmartDashboard.putNumber("Shooter/Flex Target RPM", m_targetRPM);
        SmartDashboard.putNumber("Shooter/Flex Output %", m_flexOutputPercent);
        SmartDashboard.putBoolean("Shooter/At Target RPM", 
            isAtTargetRPM(m_targetRPM, Constants.ShooterConstants.kShooterRpmTolerance));
        SmartDashboard.putNumber("Shooter/RPM Error", 
            getFlexRPM() - m_targetRPM);
        
        // Max shooter (feeder wheel)
        SmartDashboard.putNumber("Shooter/Max RPM", getMaxRPM());
        SmartDashboard.putNumber("Shooter/Max Output %", m_maxOutputPercent);
        
        // Dashboard tuning values
        SmartDashboard.putNumber("Shooter/Dashboard Target RPM", m_dashboardTargetRPM);
        
        // Current PID values (read-only display in telemetry section)
        SmartDashboard.putNumber("Shooter/Active P Gain", m_dashboardP);
        SmartDashboard.putNumber("Shooter/Active I Gain", m_dashboardI);
        SmartDashboard.putNumber("Shooter/Active D Gain", m_dashboardD);
        SmartDashboard.putNumber("Shooter/Active FF Gain", m_dashboardFF);
        SmartDashboard.putNumber("Shooter/RPM Tolerance", Constants.ShooterConstants.kShooterRpmTolerance);
    }

  public void setShooterFlexSpeed(double speed) {
    m_flexOutputPercent = speed;
    m_shooterFlex.set(speed);
  }

   public void setShooterMaxSpeed(double speed) {
    m_maxOutputPercent = speed;
    m_shooterMax.set(speed);
  }

  /**
   * Sets the SparkFlex shooter to a target RPM using the internal PID controller.
   * This ensures the flywheel maintains consistent speed.
   *
   * @param targetRPM the target RPM for the flywheel
   */
  public void setShooterFlexRPM(double targetRPM) {
    m_targetRPM = targetRPM;
    // Use the setSetpoint method with kVelocity control type
    m_flexPIDController.setSetpoint(targetRPM, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
  }

  /**
   * Gets the current RPM of the flex shooter motor.
   * 
   * @return the current RPM
   */
  public double getFlexRPM() {
    return m_shooterFlex.getEncoder().getVelocity();
  }

  /**
   * Gets the current RPM of the max shooter motor.
   * 
   * @return the current RPM
   */
  public double getMaxRPM() {
    return m_shooterMax.getEncoder().getVelocity();
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
    double flexRPM = getFlexRPM();
    return Math.abs(flexRPM - targetRPM) <= tolerance;
  }
    
}
