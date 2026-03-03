package frc.robot.subsystems;

import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants;

import frc.robot.Configs;
import com.revrobotics.ResetMode; 


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static SparkMax m_intake = new SparkMax(Constants.DriveConstants.intakeID, SparkLowLevel.MotorType.kBrushed);
  private double m_lastSpeed = 0.0;

  public Intake() {
    m_intake.configure(Configs.intake.intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Publish last commanded speed at regular rate for lightweight telemetry
    SmartDashboard.putNumber(Constants.LimelightConstants.kIntakeLastSpeedKey, m_lastSpeed);
  }

  public void setSpeed(double speed) {
    m_lastSpeed = speed;
    m_intake.set(speed);
  }
    
}
