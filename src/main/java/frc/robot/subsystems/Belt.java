package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.io.BeltIO;
import frc.robot.subsystems.io.BeltIOInputsAutoLogged;

public class Belt extends SubsystemBase {

  private final BeltIO m_io;
  private final BeltIOInputsAutoLogged m_inputs = new BeltIOInputsAutoLogged();

  public Belt(BeltIO io) {
    m_io = io;
    m_io.init(this);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Belt", m_inputs);
  }

  public void setSpeed(double speed) {
    m_io.setDutyCycle(speed);
  }
}
