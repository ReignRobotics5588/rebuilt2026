package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Simulation constants for physics modeling of robot subsystems.
 * These values define how the robot behaves in simulation.
 */
public final class SimulationConstants {
  
  /**
   * Drive subsystem simulation constants
   */
  public static final class DriveSimConstants {
    // Motor inertia (kg*m^2)
    public static final double kDriveMotorInertia = 0.025;
    public static final double kTurnMotorInertia = 0.01;
    
    // Gearing ratios
    public static final double kDriveGearing = 6.12;  // 6.12:1 for MAXSwerve
    public static final double kTurnGearing = 150.0 / 7.0;  // ~21.43:1 for MAXSwerve
    
    // Wheel properties
    public static final double kWheelRadiusInches = 2.0;  // Wheel radius in inches
    public static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);
    
    // Motor properties
    public static final double kDriveMotorMaxRPM = 5676.0;  // NEO motor max RPM
    public static final double kTurnMotorMaxRPM = 1150.0;  // NEO 550 motor max RPM
    
    // Friction and drag
    public static final double kDriveVoltageCompensation = 12.0;  // Volts
    public static final double kWheelCOF = 1.0;  // Coefficient of friction
  }
  
  /**
   * Shooter subsystem simulation constants
   */
  public static final class ShooterSimConstants {
    // SparkFlex flywheel motor inertia
    public static final double kFlexMotorInertia = 0.04;
    public static final double kMaxMotorInertia = 0.02;
    
    // Gearing
    public static final double kFlexGearing = 1.0;  // Direct drive
    public static final double kMaxGearing = 1.0;   // Direct drive
    
    // Motor specs
    public static final double kFlexMotorMaxRPM = 6784.0;  // SparkFlex max RPM
    public static final double kMaxMotorMaxRPM = 5676.0;   // SparkMax max RPM
    
    // Flywheel moment of inertia (affects spin-up time)
    public static final double kFlywheelMOI = 0.08;  // kg*m^2
  }
  
  /**
   * Intake subsystem simulation constants
   */
  public static final class IntakeSimConstants {
    public static final double kMotorInertia = 0.02;
    public static final double kGearing = 1.0;
    public static final double kMotorMaxRPM = 5676.0;
  }
  
  /**
   * Belt subsystem simulation constants
   */
  public static final class BeltSimConstants {
    public static final double kMotorInertia = 0.02;
    public static final double kGearing = 1.0;
    public static final double kMotorMaxRPM = 5676.0;
  }
  
  /**
   * Vision subsystem simulation constants
   */
  public static final class VisionSimConstants {
    // Camera field of view
    public static final double kCameraFOVDegrees = 63.3;  // Limelight 2 horizontal FOV
    
    // Detection parameters
    public static final double kAprilTagDetectionRangeMeters = 5.0;  // Max detection distance
    public static final double kAprilTagSimulationNoise = 0.02;  // Measurement noise in degrees
    
    // Latency simulation
    public static final double kCameraLatencyMs = 11.0;  // Typical camera latency
  }
}
