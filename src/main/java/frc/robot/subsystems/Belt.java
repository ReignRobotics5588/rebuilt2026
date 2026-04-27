package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Belt extends SubsystemBase {
  private final BeltIO io;
  private final BeltIOInputsAutoLogged inputs = new BeltIOInputsAutoLogged();

  public Belt(BeltIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Belt", inputs);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }
}
