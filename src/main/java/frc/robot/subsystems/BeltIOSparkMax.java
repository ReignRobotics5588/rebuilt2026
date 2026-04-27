package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.Configs;
import frc.robot.Constants;

public class BeltIOSparkMax implements BeltIO {
  private final SparkMax motor = new SparkMax(Constants.BeltConstants.kBeltCanId,
      SparkLowLevel.MotorType.kBrushless);

  public BeltIOSparkMax() {
    motor.configure(Configs.belt.belt_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(BeltIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
