package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Shooter extends SubsystemBase {

  private final SparkFlex m_flywheelSpark = new SparkFlex(DriveConstants.flywheelID, MotorType.kBrushless);
  private final SparkMax m_indexerSpark = new SparkMax(DriveConstants.indexerID, MotorType.kBrushless);

  private final SmartMotorController m_flywheel;
  private final SmartMotorController m_indexer;

  private double m_targetRPM = 0.0;

  public Shooter() {
    SmartMotorControllerConfig flywheelConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(1.0)
        .withClosedLoopController(
            ShooterConstants.kFlywheelP,
            ShooterConstants.kFlywheelI,
            ShooterConstants.kFlywheelD)
        .withSimClosedLoopController(0.1, 0.0, 0.0)
        .withFeedforward(new SimpleMotorFeedforward(0.0, ShooterConstants.kFlywheelFF, 0.0))
        .withSimFeedforward(new SimpleMotorFeedforward(0.0, 0.0169, 0.0))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(ShooterConstants.kFlywheelCurrentLimit))
        .withTelemetry("Shooter/Flywheel", TelemetryVerbosity.HIGH);

    m_flywheel = new SparkWrapper(m_flywheelSpark, DCMotor.getNeoVortex(1), flywheelConfig);

    SmartMotorControllerConfig indexerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(1.0)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("Shooter/Indexer", TelemetryVerbosity.LOW);

    m_indexer = new SparkWrapper(m_indexerSpark, DCMotor.getNEO(1), indexerConfig);
  }

  @Override
  public void periodic() {
    m_flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_flywheel.simIterate();
  }

  public void setFlywheelRPM(double targetRPM) {
    m_targetRPM = targetRPM;
    m_flywheel.setVelocity(RPM.of(targetRPM));
  }

  public void setFlywheelSpeed(double speed) {
    m_targetRPM = 0.0;
    m_flywheel.setDutyCycle(speed);
  }

  public double getFlywheelRPM() {
    return m_flywheel.getMechanismVelocity().in(RPM);
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public boolean isAtTargetRPM(double targetRPM, double tolerance) {
    return Math.abs(getFlywheelRPM() - targetRPM) <= tolerance;
  }

  public void setIndexerSpeed(double speed) {
    m_indexer.setDutyCycle(speed);
  }
}
