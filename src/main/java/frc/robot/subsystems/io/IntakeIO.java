package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public double appliedDutyCycle = 0.0;
    public double currentAmps = 0.0;
  }

  default void init(SubsystemBase subsystem) {}

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setDutyCycle(double speed) {}
}
