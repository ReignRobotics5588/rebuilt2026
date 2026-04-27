package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ModuleConstants;

public class ModuleIOSim implements ModuleIO {
  // MAXSwerve drive reduction: 45T bevel * 22T spur / (13T pinion * 15T bevel pinion)
  private static final double DRIVE_GEAR_RATIO =
      (45.0 * 22) / (ModuleConstants.kDrivingMotorPinionTeeth * 15);
  // MAXSwerve standard turning reduction
  private static final double TURN_GEAR_RATIO = 9424.0 / 203.0;

  private final DCMotorSim driveSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, DRIVE_GEAR_RATIO),
      DCMotor.getNEO(1));
  private final DCMotorSim turnSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, TURN_GEAR_RATIO),
      DCMotor.getNEO(1));

  private final PIDController drivePID = new PIDController(1.5, 0.0, 0.0);
  private final PIDController turnPID = new PIDController(20.0, 0.0, 0.0);

  private double driveSetpointMps = 0.0;
  private double turnSetpointRad = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double wheelRadius = ModuleConstants.kWheelDiameterMeters / 2.0;

    // Drive: PID on wheel linear velocity
    double driveVelocityMps = driveSim.getAngularVelocityRadPerSec() * wheelRadius;
    driveAppliedVolts = MathUtil.clamp(
        drivePID.calculate(driveVelocityMps, driveSetpointMps), -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);

    // Turn: PID on module angle (wrapped to [-π, π])
    double turnAngleRad = MathUtil.angleModulus(turnSim.getAngularPositionRad());
    turnAppliedVolts = MathUtil.clamp(
        turnPID.calculate(turnAngleRad, turnSetpointRad), -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);

    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.turnConnected = true;
    inputs.drivePositionMeters = driveSim.getAngularPositionRad() * wheelRadius;
    inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * wheelRadius;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();
    inputs.turnAbsolutePosition = new Rotation2d(MathUtil.angleModulus(turnSim.getAngularPositionRad()));
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = turnSim.getCurrentDrawAmps();
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec) {
    driveSetpointMps = velocityMetersPerSec;
  }

  @Override
  public void setTurnPosition(double positionRadians) {
    turnSetpointRad = positionRadians;
  }
}
