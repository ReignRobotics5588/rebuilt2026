package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.io.ShooterIO;
import frc.robot.subsystems.io.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {

  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();
  private double m_targetRPM = 0.0;

  public Shooter(ShooterIO io) {
    m_io = io;
    m_io.init(this);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);
    Logger.recordOutput("Shooter/TargetRPM", m_targetRPM);
  }

  @Override
  public void simulationPeriodic() {
    m_io.simIterate();
  }

  public void setFlywheelRPM(double targetRPM) {
    m_targetRPM = targetRPM;
    m_io.setFlywheelRPM(targetRPM);
  }

  public void setFlywheelSpeed(double speed) {
    m_targetRPM = 0.0;
    m_io.setFlywheelDutyCycle(speed);
  }

  public double getFlywheelRPM() {
    return m_inputs.flywheelVelocityRPM;
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public boolean isAtTargetRPM(double targetRPM, double tolerance) {
    return Math.abs(getFlywheelRPM() - targetRPM) <= tolerance;
  }

  public void setIndexerSpeed(double speed) {
    m_io.setIndexerDutyCycle(speed);
  }
}
