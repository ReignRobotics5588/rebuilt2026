/**
 * PID CONTROL IMPLEMENTATION FOR SHOOTER FLYWHEEL
 * ================================================
 * 
 * OVERVIEW:
 * The SparkFlex shooter motor is now configured with internal PID feedback control
 * to maintain a consistent target RPM automatically. This ensures the flywheel
 * speed stays stable even under load variations.
 * 
 * 
 * KEY COMPONENTS:
 * ===============
 * 
 * 1. CONSTANTS (ShooterConstants)
 * --------------------------------
 * - kShooterTargetRPM = 3000.0        // Target RPM for the flywheel
 * - kShooterRpmTolerance = 200.0      // Acceptable RPM variation (±200)
 * - kShooterFlexP = 0.0005            // Proportional gain
 * - kShooterFlexI = 0.0               // Integral gain (disabled for now)
 * - kShooterFlexD = 0.0               // Derivative gain (disabled for now)
 * - kShooterFlexFF = 0.000156         // Feed-forward (1/6400 for Spark motor)
 * 
 * 
 * 2. SHOOTER SUBSYSTEM METHODS
 * -----------------------------
 * 
 * setShooterFlexRPM(double targetRPM)
 *   - Sets the SparkFlex to maintain a specific RPM using PID control
 *   - Usage: shooter.setShooterFlexRPM(3000);
 *   - The internal PID controller automatically adjusts output to reach target
 *   - Much more stable than percent output control
 * 
 * getFlexRPM()
 *   - Returns the current RPM of the SparkFlex motor
 *   - Useful for monitoring and logging
 * 
 * isAtTargetRPM(double targetRPM, double tolerance)
 *   - Checks if the flywheel is within tolerance of target RPM
 *   - Returns true if both motors are at speed
 * 
 * 
 * 3. CONFIGURATION (Configs.java)
 * --------------------------------
 * The SparkFlex is configured with:
 * - Closed loop control using primary encoder (internal Hall effect)
 * - Velocity feedback sensor
 * - PID gains applied through SparkFlexConfig
 * - Output range clamped to [-1.0, 1.0]
 * 
 * 
 * USAGE EXAMPLE:
 * ==============
 * 
 * In ShooterBeltCommand:
 * ----------------------
 * @Override
 * public void initialize() {
 *     // Start shooter with PID control at target RPM
 *     m_shooter.setShooterFlexRPM(Constants.ShooterConstants.kShooterTargetRPM);
 *     m_beltStarted = false;
 * }
 * 
 * @Override
 * public void execute() {
 *     // Wait for shooter to reach target RPM
 *     if (!m_beltStarted && m_shooter.isAtTargetRPM(
 *         Constants.ShooterConstants.kShooterTargetRPM,
 *         Constants.ShooterConstants.kShooterRpmTolerance)) {
 *         // Once at speed, engage belt
 *         m_belt.setSpeed(Constants.BeltConstants.kBeltSpeed);
 *         m_beltStarted = true;
 *     }
 * }
 * 
 * 
 * TUNING GUIDE:
 * =============
 * 
 * If the flywheel is too slow to reach target RPM:
 *   - Increase kShooterFlexP (proportional gain)
 *   - Or increase kShooterFlexFF (feed-forward)
 * 
 * If the flywheel overshoots and oscillates:
 *   - Decrease kShooterFlexP
 *   - Or increase kShooterFlexD (derivative damping)
 * 
 * If the RPM drifts slowly over time:
 *   - Add small kShooterFlexI value (e.g., 0.0001)
 * 
 * Starting values that work well:
 *   - P: 0.0003 to 0.0010
 *   - I: 0.0 (keep disabled unless needed)
 *   - D: 0.0 (keep disabled unless needed)
 *   - FF: 0.000156 (1/6400 for Spark motors)
 * 
 * 
 * ADVANTAGES:
 * ===========
 * ✓ Automatic speed regulation without manual code intervention
 * ✓ Stable performance across battery voltage variations
 * ✓ Consistent shooter accuracy due to stable RPM
 * ✓ Less responsive to external load changes (game piece friction)
 * ✓ Simpler command code - no need for percent output workarounds
 * 
 * 
 * NOTE:
 * =====
 * The SparkMax shooter motor currently uses percent output control.
 * Consider upgrading it to RPM control as well for more consistent performance.
 * You could add a setShooterMaxRPM() method similar to setShooterFlexRPM().
 */