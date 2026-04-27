package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class intake {
        public static final SparkMaxConfig intake_config = new SparkMaxConfig();

        static {
            intake_config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(60);
        }
    }

    public static final class flywheel {
        public static final SparkFlexConfig flywheel_config = new SparkFlexConfig();

        static {
            flywheel_config
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(Constants.ShooterConstants.kFlywheelCurrentLimit);

            flywheel_config.encoder
                    .velocityConversionFactor(1.0);

            flywheel_config.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(Constants.ShooterConstants.kFlywheelP,
                         Constants.ShooterConstants.kFlywheelI,
                         Constants.ShooterConstants.kFlywheelD)
                    .outputRange(-1.0, 1.0);
        }
    }

    public static final class indexer {
        public static final SparkMaxConfig indexer_config = new SparkMaxConfig();

        static {
            indexer_config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
        }
    }

    public static final class belt {
        public static final SparkMaxConfig belt_config = new SparkMaxConfig();

        static {
            belt_config
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
        }
    }
}
