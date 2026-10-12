package frc.robot.subsystems.io;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class BeltIOYAMS implements BeltIO {

  private SmartMotorController m_motor;

  @Override
  public void init(SubsystemBase subsystem) {
    SparkMax spark = new SparkMax(DriveConstants.beltID, MotorType.kBrushless);

    SmartMotorControllerConfig config = new SmartMotorControllerConfig(subsystem)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(1.0)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("Belt", TelemetryVerbosity.LOW);

    m_motor = new SparkWrapper(spark, DCMotor.getNEO(1), config);
  }

  @Override
  public void updateInputs(BeltIOInputs inputs) {
    inputs.appliedDutyCycle = m_motor.getDutyCycle();
    inputs.currentAmps = m_motor.getStatorCurrent().in(Amps);
  }

  @Override
  public void setDutyCycle(double speed) {
    m_motor.setDutyCycle(speed);
  }
}
