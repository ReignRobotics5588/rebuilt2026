package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface BeltIO {
  @AutoLog
  class BeltIOInputs {
    public double appliedDutyCycle = 0.0;
    public double currentAmps = 0.0;
  }

  default void init(SubsystemBase subsystem) {}

  default void updateInputs(BeltIOInputs inputs) {}

  default void setDutyCycle(double speed) {}
}
