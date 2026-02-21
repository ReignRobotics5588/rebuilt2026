package frc.robot.subsystems;

import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public static SparkMax m_intake = new SparkMax(Constants.DriveConstants.intakeID, SparkLowLevel.MotorType.kBrushed);

  public Intake() {
    m_intake.configure(Configs.arm.intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    System.out.println("setting intake");

    m_intake.set(speed);
  }
    
}
