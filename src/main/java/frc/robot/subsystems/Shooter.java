package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double m_targetRPM = 0.0;

  private double m_dashboardP = Constants.ShooterConstants.kFlywheelP;
  private double m_dashboardI = Constants.ShooterConstants.kFlywheelI;
  private double m_dashboardD = Constants.ShooterConstants.kFlywheelD;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setFlywheelSpeed(double speed) {
    io.setFlywheelSpeed(speed);
  }

  public void setIndexerSpeed(double speed) {
    io.setIndexerSpeed(speed);
  }

  public void setFlywheelRPM(double targetRPM) {
    m_targetRPM = targetRPM;
    io.setFlywheelRPM(targetRPM);
  }

  @AutoLogOutput(key = "Shooter/CurrentRPM")
  public double getFlywheelRPM() {
    return inputs.flywheelVelocityRPM;
  }

  @AutoLogOutput(key = "Shooter/TargetRPM")
  public double getTargetRPM() {
    return m_targetRPM;
  }

  public boolean isAtTargetRPM(double targetRPM, double tolerance) {
    return Math.abs(inputs.flywheelVelocityRPM - targetRPM) <= tolerance;
  }

  /**
   * Update PID gains and target RPM from dashboard. Called every cycle from RobotContainer.
   */
  public void updatePIDFromDashboard(double newP, double newI, double newD, double newTargetRPM) {
    if (newP != m_dashboardP || newI != m_dashboardI || newD != m_dashboardD) {
      m_dashboardP = newP;
      m_dashboardI = newI;
      m_dashboardD = newD;
      io.updateFlywheelPID(newP, newI, newD);
    }
    m_targetRPM = newTargetRPM;
  }
}
