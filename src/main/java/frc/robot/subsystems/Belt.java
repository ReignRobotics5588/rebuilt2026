package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants;

import frc.robot.Configs;
import com.revrobotics.ResetMode;

public class Belt extends SubsystemBase {
  /** Creates a new Belt. */
  private static SparkMax m_belt = new SparkMax(Constants.DriveConstants.beltID, SparkLowLevel.MotorType.kBrushed);
  private double m_lastSpeed = 0.0;

  public Belt() {
    m_belt.configure(Configs.intake.intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  public void setSpeed(double speed) {
    m_lastSpeed = speed;
    m_belt.set(speed);
  }

  @Override
  public void periodic() {
    // Publish last commanded belt speed at a regular rate
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(Constants.LimelightConstants.kBeltLastSpeedKey, m_lastSpeed);
  }
    
}
