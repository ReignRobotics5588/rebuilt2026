package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double flywheelPositionRotations = 0.0;
    public double flywheelVelocityRPM = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelAppliedDutyCycle = 0.0;
    public double flywheelCurrentAmps = 0.0;
    public double indexerAppliedDutyCycle = 0.0;
    public double indexerCurrentAmps = 0.0;
  }

  default void init(SubsystemBase subsystem) {}

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setFlywheelRPM(double rpm) {}

  default void setFlywheelDutyCycle(double speed) {}

  /** Open-loop voltage, used for SysId characterization. */
  default void setFlywheelVoltage(double volts) {}

  default void setIndexerDutyCycle(double speed) {}

  default void simIterate() {}
}
