package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Configs;

public class ModuleIOSparkMax implements ModuleIO {
  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  private final double chassisAngularOffset;

  @SuppressWarnings("removal")
  public ModuleIOSparkMax(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    this.chassisAngularOffset = chassisAngularOffset;
    driveSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    turnSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getAbsoluteEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    driveSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    turnSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    driveEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.turnConnected = true;
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSpark.getAppliedOutput() * driveSpark.getBusVoltage();
    inputs.driveCurrentAmps = driveSpark.getOutputCurrent();
    // Subtract chassis offset so subsystem works in robot-relative coordinates
    inputs.turnAbsolutePosition = new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset);
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSpark.getAppliedOutput() * turnSpark.getBusVoltage();
    inputs.turnCurrentAmps = turnSpark.getOutputCurrent();
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec) {
    driveController.setSetpoint(velocityMetersPerSec, ControlType.kVelocity);
  }

  @Override
  public void setTurnPosition(double positionRadians) {
    // Add chassis offset back before commanding the motor (raw encoder space)
    turnController.setSetpoint(positionRadians + chassisAngularOffset, ControlType.kPosition);
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }
}
