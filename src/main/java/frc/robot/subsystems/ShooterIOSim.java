package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  // Neo Vortex direct-drive flywheel: 60 RPM/s ≈ 6.28 rad/s, j=0.01 kg*m²
  private final DCMotorSim flywheelSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.01, 1.0),
      DCMotor.getNeoVortex(1));

  private final PIDController flywheelPID = new PIDController(0.001, 0.0, 0.0);
  private double targetRPM = 0.0;
  private double indexerAppliedVolts = 0.0;
  private boolean useVelocityControl = false;
  private double manualSpeed = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double flywheelVolts;
    if (useVelocityControl) {
      double currentRPM = flywheelSim.getAngularVelocityRadPerSec() * 60.0 / (2 * Math.PI);
      flywheelVolts = MathUtil.clamp(flywheelPID.calculate(currentRPM, targetRPM), -12.0, 12.0);
    } else {
      flywheelVolts = manualSpeed * 12.0;
    }
    flywheelSim.setInputVoltage(flywheelVolts);
    flywheelSim.update(0.02);

    inputs.flywheelVelocityRPM = flywheelSim.getAngularVelocityRadPerSec() * 60.0 / (2 * Math.PI);
    inputs.flywheelAppliedVolts = flywheelVolts;
    inputs.flywheelCurrentAmps = flywheelSim.getCurrentDrawAmps();
    inputs.indexerAppliedVolts = indexerAppliedVolts;
    inputs.indexerCurrentAmps = 0.0;
  }

  @Override
  public void setFlywheelRPM(double rpm) {
    targetRPM = rpm;
    useVelocityControl = true;
  }

  @Override
  public void setFlywheelSpeed(double speed) {
    manualSpeed = speed;
    useVelocityControl = false;
  }

  @Override
  public void setIndexerSpeed(double speed) {
    indexerAppliedVolts = speed * 12.0;
  }
}
