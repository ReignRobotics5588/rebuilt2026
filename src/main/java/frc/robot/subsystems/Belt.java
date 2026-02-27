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

  public Belt() {
    m_belt.configure(Configs.intake.intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  public void setSpeed(double speed) {
    System.out.println("setting belt");

    m_belt.set(speed);
  }
    
}
