package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.io.IntakeIO;
import frc.robot.subsystems.io.IntakeIOInputsAutoLogged;

public class Intake extends SubsystemBase {

  private final IntakeIO m_io;
  private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    m_io = io;
    m_io.init(this);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Intake", m_inputs);
  }

  public void setSpeed(double speed) {
    m_io.setDutyCycle(speed);
  }
}
