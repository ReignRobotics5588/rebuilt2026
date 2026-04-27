package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class BeltIOSim implements BeltIO {
  private final DCMotorSim motorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.005, 1.0),
      DCMotor.getNEO(1));
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(BeltIOInputs inputs) {
    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.velocityRPM = motorSim.getAngularVelocityRadPerSec() * 60.0 / (2 * Math.PI);
  }

  @Override
  public void setSpeed(double speed) {
    appliedVolts = speed * 12.0;
  }
}
