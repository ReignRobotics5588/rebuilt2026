package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Configs;
import frc.robot.Constants;

public class ShooterIOSparkFlex implements ShooterIO {
  private final SparkFlex flywheel = new SparkFlex(Constants.ShooterConstants.kFlywheelCanId,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkMax indexer = new SparkMax(Constants.ShooterConstants.kIndexerCanId,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkClosedLoopController flywheelPID = flywheel.getClosedLoopController();

  private double cachedP = Constants.ShooterConstants.kFlywheelP;
  private double cachedI = Constants.ShooterConstants.kFlywheelI;
  private double cachedD = Constants.ShooterConstants.kFlywheelD;

  public ShooterIOSparkFlex() {
    flywheel.configure(Configs.flywheel.flywheel_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    indexer.configure(Configs.indexer.indexer_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelVelocityRPM = flywheel.getEncoder().getVelocity();
    inputs.flywheelAppliedVolts = flywheel.getAppliedOutput() * flywheel.getBusVoltage();
    inputs.flywheelCurrentAmps = flywheel.getOutputCurrent();
    inputs.indexerAppliedVolts = indexer.getAppliedOutput() * indexer.getBusVoltage();
    inputs.indexerCurrentAmps = indexer.getOutputCurrent();
  }

  @Override
  public void setFlywheelRPM(double rpm) {
    flywheelPID.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void setFlywheelSpeed(double speed) {
    flywheel.set(speed);
  }

  @Override
  public void setIndexerSpeed(double speed) {
    indexer.set(speed);
  }

  @Override
  public void updateFlywheelPID(double p, double i, double d) {
    if (p == cachedP && i == cachedI && d == cachedD) return;
    cachedP = p;
    cachedI = i;
    cachedD = d;
    SparkFlexConfig cfg = new SparkFlexConfig();
    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(Constants.ShooterConstants.kFlywheelCurrentLimit);
    cfg.encoder.velocityConversionFactor(1.0);
    cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(p, i, d)
        .outputRange(-1.0, 1.0);
    flywheel.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
