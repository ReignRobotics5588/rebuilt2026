package frc.robot.subsystems;

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

public class Belt extends SubsystemBase {

  private final SparkMax m_beltSpark = new SparkMax(DriveConstants.beltID, MotorType.kBrushless);
  private final SmartMotorController m_motor;

  public Belt() {
    SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(1.0)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("Belt", TelemetryVerbosity.LOW);

    m_motor = new SparkWrapper(m_beltSpark, DCMotor.getNEO(1), config);
  }

  public void setSpeed(double speed) {
    m_motor.setDutyCycle(speed);
  }
}
