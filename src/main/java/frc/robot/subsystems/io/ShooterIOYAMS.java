package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
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
    SparkFlex flywheelSpark = new SparkFlex(ShooterConstants.kFlywheelCanId, MotorType.kBrushless);
    SparkMax indexerSpark = new SparkMax(ShooterConstants.kIndexerCanId, MotorType.kBrushless);

    SmartMotorControllerConfig flywheelConfig = new SmartMotorControllerConfig(subsystem)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(1.0)
        .withClosedLoopController(
            ShooterConstants.kFlywheelP,
            ShooterConstants.kFlywheelI,
            ShooterConstants.kFlywheelD)
        .withFeedforward(new SimpleMotorFeedforward(0.0, ShooterConstants.kFlywheelFF, 0.0))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(ShooterConstants.kFlywheelCurrentLimit))
        .withTelemetry("Shooter/Flywheel", Constants.kTelemetryVerbosity);

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
    inputs.flywheelPositionRotations = m_flywheel.getMechanismPosition().in(Rotations);
    inputs.flywheelVelocityRPM = m_flywheel.getMechanismVelocity().in(RPM);
    inputs.flywheelAppliedVolts = m_flywheel.getVoltage().in(Volts);
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
  public void setFlywheelVoltage(double volts) {
    m_flywheel.setVoltage(Volts.of(volts));
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
