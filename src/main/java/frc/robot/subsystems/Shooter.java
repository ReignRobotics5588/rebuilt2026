package frc.robot.subsystems;

import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants;

import frc.robot.Configs;
import com.revrobotics.ResetMode; 

public class Shooter extends SubsystemBase {

    public static SparkFlex m_shooterFlex = new SparkFlex(Constants.DriveConstants.shooterFlexID, SparkLowLevel.MotorType.kBrushed);
    public static SparkMax m_shooterMax = new SparkMax(Constants.DriveConstants.shooterMaxID, SparkLowLevel.MotorType.kBrushed);

    public Shooter() {
        m_shooterFlex.configure(Configs.shooterFlex.shooterFlex_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterMax.configure(Configs.shooterMax.shooterMax_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  public void setShooterFlexSpeed(double speed) {
    System.out.println("setting shooter flex");

    m_shooterFlex.set(speed);
  }

   public void setShooterMaxSpeed(double speed) {
    System.out.println("setting shooter max");

    m_shooterMax.set(speed);
  }
    
}
