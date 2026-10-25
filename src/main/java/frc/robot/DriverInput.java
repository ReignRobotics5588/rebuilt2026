package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;

/**
 * Shapes the driver's sticks into smooth drive commands (fractions of max speed).
 *
 * <p>Translation is handled as one vector so diagonals keep their direction: a
 * circular deadband, then a response curve on the magnitude, then a limit on how
 * fast the commanded velocity can change (acceleration). Rotation gets the same
 * deadband, curve and rate limit on its own axis.
 *
 * <p>Call {@link #update()} once per loop before the scheduler runs; commands
 * read the cached values.
 */
public class DriverInput {
  private final CommandXboxController m_controller;
  private final SlewRateLimiter m_rotationLimiter =
      new SlewRateLimiter(OIConstants.kRotationAccelPerSec);

  private Translation2d m_translation = Translation2d.kZero;
  private double m_rotation = 0.0;
  private double m_lastTime = Timer.getFPGATimestamp();

  public DriverInput(CommandXboxController controller) {
    m_controller = controller;
  }

  public void update() {
    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastTime;
    m_lastTime = now;

    // Don't carry a stale velocity into the next enable
    if (DriverStation.isDisabled()) {
      m_translation = Translation2d.kZero;
      m_rotation = 0.0;
      m_rotationLimiter.reset(0.0);
      return;
    }

    // Stick up/left is negative on Xbox controllers; field +X is forward, +Y is left
    Translation2d stick = new Translation2d(-m_controller.getLeftY(), -m_controller.getLeftX());
    double magnitude = shape(Math.min(stick.getNorm(), 1.0), OIConstants.kTranslationExponent);
    Translation2d target = magnitude == 0.0
        ? Translation2d.kZero
        : stick.div(stick.getNorm()).times(magnitude);
    m_translation = limitRate(m_translation, target, dt);

    m_rotation = m_rotationLimiter.calculate(
        Math.copySign(shape(Math.abs(m_controller.getRightX()), OIConstants.kRotationExponent),
            -m_controller.getRightX()));
  }

  /** Forward speed, fraction of max (field-relative). */
  public double forward() {
    return m_translation.getX();
  }

  /** Left speed, fraction of max (field-relative). */
  public double left() {
    return m_translation.getY();
  }

  /** Counter-clockwise rotation, fraction of max. */
  public double rotation() {
    return m_rotation;
  }

  // Deadband (rescaled so output starts at 0 just past it), then the response curve
  private static double shape(double value, double exponent) {
    return Math.pow(MathUtil.applyDeadband(value, OIConstants.kDriveDeadband), exponent);
  }

  // Moves current toward target by at most accel*dt (or decel*dt when slowing down)
  private static Translation2d limitRate(Translation2d current, Translation2d target, double dt) {
    double rate = target.getNorm() < current.getNorm()
        ? OIConstants.kTranslationDecelPerSec
        : OIConstants.kTranslationAccelPerSec;
    double maxStep = rate * dt;
    Translation2d delta = target.minus(current);
    if (delta.getNorm() <= maxStep) {
      return target;
    }
    return current.plus(delta.times(maxStep / delta.getNorm()));
  }
}
