package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.io.ShooterIO;
import frc.robot.subsystems.io.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {

  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();
  private final SysIdRoutine m_sysId;
  private double m_targetRPM = 0.0;

  public Shooter(ShooterIO io) {
    m_io = io;
    m_io.init(this);

    // Data is recorded by AdvantageKit (Shooter/*); load the .wpilog in the SysId tool
    m_sysId = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null,
            state -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(volts -> m_io.setFlywheelVoltage(volts.in(Volts)), null, this));
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

  /** Flywheel RPM for a shot from this far (meters) from the hub center. */
  public static double getRPMForDistance(double distanceMeters) {
    return ShooterConstants.kDistanceToRPM.get(distanceMeters);
  }

  /** True once the flywheel is fast enough to start feeding for this target. */
  public boolean isReadyToFeed(double targetRPM) {
    return getFlywheelRPM() >= targetRPM - ShooterConstants.kFeedRpmMargin;
  }

  public void setIndexerSpeed(double speed) {
    m_io.setIndexerDutyCycle(speed);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysId.dynamic(direction);
  }
}
