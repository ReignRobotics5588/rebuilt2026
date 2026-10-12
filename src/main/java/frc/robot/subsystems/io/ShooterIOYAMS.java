package frc.robot.subsystems.io;

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

public class ShooterIOYAMS implements ShooterIO {

  private SmartMotorController m_flywheel;
  private SmartMotorController m_indexer;

  @Override
  public void init(SubsystemBase subsystem) {
    SparkFlex flywheelSpark = new SparkFlex(DriveConstants.flywheelID, MotorType.kBrushless);
    SparkMax indexerSpark = new SparkMax(DriveConstants.indexerID, MotorType.kBrushless);

    SmartMotorControllerConfig flywheelConfig = new SmartMotorControllerConfig(subsystem)
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

    m_flywheel = new SparkWrapper(flywheelSpark, DCMotor.getNeoVortex(1), flywheelConfig);

    SmartMotorControllerConfig indexerConfig = new SmartMotorControllerConfig(subsystem)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(1.0)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("Shooter/Indexer", TelemetryVerbosity.LOW);

    m_indexer = new SparkWrapper(indexerSpark, DCMotor.getNEO(1), indexerConfig);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    m_flywheel.updateTelemetry();
    inputs.flywheelVelocityRPM = m_flywheel.getMechanismVelocity().in(RPM);
    inputs.flywheelAppliedDutyCycle = m_flywheel.getDutyCycle();
    inputs.flywheelCurrentAmps = m_flywheel.getStatorCurrent().in(Amps);
    inputs.indexerAppliedDutyCycle = m_indexer.getDutyCycle();
    inputs.indexerCurrentAmps = m_indexer.getStatorCurrent().in(Amps);
  }

  @Override
  public void setFlywheelRPM(double rpm) {
    m_flywheel.setVelocity(RPM.of(rpm));
  }

  @Override
  public void setFlywheelDutyCycle(double speed) {
    m_flywheel.setDutyCycle(speed);
  }

  @Override
  public void setIndexerDutyCycle(double speed) {
    m_indexer.setDutyCycle(speed);
  }

  @Override
  public void simIterate() {
    m_flywheel.simIterate();
  }
}
