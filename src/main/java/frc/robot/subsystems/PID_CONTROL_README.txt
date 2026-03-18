/**
 * FLYWHEEL PID TUNING GUIDE
 * =========================
 * 
 * REV Robotics Velocity PID Control Overview
 * ===========================================
 * The SparkFlex flywheel uses REV's internal closed-loop velocity controller to maintain
 * a constant RPM target. This is far superior to open-loop percent output because:
 * - It automatically compensates for battery voltage sag
 * - It maintains speed even when game pieces are loaded
 * - It reaches target RPM faster and more consistently
 * 
 * How setSetpoint() with kVelocity Works:
 * ----------------------------------------
 * 1. You call: m_shooter.setFlywheelRPM(targetRPM)
 * 2. Internally: m_flywheelPIDController.setSetpoint(targetRPM, ControlType.kVelocity)
 * 3. The SparkFlex measures current RPM via the built-in encoder
 * 4. It calculates error = targetRPM - currentRPM
 * 5. The PID controller continuously adjusts motor output using:
 *    Output = (P * error) + (I * sum_of_errors) + (D * rate_of_error_change)
 * 6. Output is clamped to [-1.0, 1.0] (0-100% power)
 * 
 * 
 * CURRENT TUNING VALUES (in Constants.ShooterConstants):
 * =======================================================
 * kShooterTargetRPM     = 3000.0       // Target flywheel RPM
 * kShooterRpmTolerance  = 50.0         // How close to target before engaging indexer
 * kFlywheelP            = 0.0016869    // Proportional gain (acceleration/response)
 * kFlywheelI            = 0.0          // Integral gain (currently disabled)
 * kFlywheelD            = 0.0012       // Derivative gain (damping/smoothing)
 * kFlywheelFF           = 0.000156     // Feed-forward (1/6400 for SparkFlex)
 * 
 * 
 * HOW TO MAKE THE SYSTEM SPEED UP FASTER:
 * ========================================
 * 
 * OPTION 1: Increase Proportional Gain (P)
 * -----------------------------------------
 * The P term is what makes the motor accelerate faster when far from target.
 * 
 * Current: kFlywheelP = 0.0016869
 * Try:     kFlywheelP = 0.002 to 0.0025 (10-50% increase)
 * 
 * Effect: Motor will accelerate faster initially, but watch for oscillation.
 * Warning: Too high P causes overshoot and oscillation (ringing).
 * 
 * How to adjust safely:
 *   1. Increase by 0.0001 at a time
 *   2. Test with SmartDashboard: set dashboard target RPM and watch the ramp
 *   3. Look at "Flywheel Output %" - it should reach 100% quickly if needed
 *   4. If RPM overshoots and oscillates, DECREASE P back down
 * 
 * 
 * OPTION 2: Increase Derivative Gain (D)
 * ----------------------------------------
 * The D term prevents overshoot and damping oscillations while still allowing fast acceleration.
 * 
 * Current: kFlywheelD = 0.0012
 * Try:     kFlywheelD = 0.002 to 0.003 (50-150% increase)
 * 
 * Effect: Smoother acceleration, reduces oscillation, allows higher P.
 * This is the RECOMMENDED approach for faster speed-up without instability.
 * 
 * How to adjust:
 *   1. Increase D first before increasing P
 *   2. Small increases in D significantly smooth response
 *   3. Monitor: when D increases, ramp should be smoother and less bouncy
 * 
 * 
 * OPTION 3: Add Feed-Forward Component
 * ----------------------------------------
 * Currently, kFlywheelFF = 0.000156 (typically 1/6400 for SparkFlex).
 * REV Robotics 2026 deprecated explicit FF, but you can add it via P tuning.
 * 
 * To enable faster speed-up:
 *   - Increase kFlywheelP (handles the feed-forward effect)
 *   - The system will naturally accelerate faster at startup
 * 
 * 
 * RECOMMENDED TUNING SEQUENCE:
 * ============================
 * 
 * Step 1: Test Current Performance
 * ---------
 * 1. Open SmartDashboard
 * 2. Go to "PID Shooter Testing" section
 * 3. Set "Dashboard Target RPM" to 3000
 * 4. Observe:
 *    - "Flywheel Output %" (should reach high % quickly)
 *    - "Flywheel RPM" (smooth ramp to 3000?)
 *    - "RPM Error" (goes from 3000 to 0 as flywheel speeds up)
 * 5. Note the time it takes to reach target (use driver station timer)
 * 
 * Step 2: Increase Damping (D Gain)
 * ---------
 * If acceleration looks jerky or bouncy:
 *   1. Change kFlywheelD from 0.0012 to 0.002
 *   2. Save and deploy
 *   3. Run same test - smoother ramp?
 *   4. If yes, can increase even more (try 0.0025 or 0.003)
 *   5. If too much smoothing, dial back to 0.0018
 * 
 * Step 3: Increase Responsiveness (P Gain)
 * ---------
 * If motor is still slow to accelerate:
 *   1. Change kFlywheelP from 0.0016869 to 0.002
 *   2. Save and deploy
 *   3. Run same test - faster acceleration?
 *   4. If no oscillation, try 0.0025
 *   5. If oscillates (RPM jumps up and down), reduce back
 * 
 * Step 4: Add Integral (I Gain) Only If Needed
 * ---------
 * If flywheel can't quite maintain target RPM and slowly drifts down:
 *   1. Change kFlywheelI from 0.0 to 0.00001
 *   2. Save and deploy
 *   3. Watch "RPM Error" - should stay near 0
 *   4. If still drifting, try 0.00005 or 0.0001
 *   WARNING: Too much I causes oscillation. Start tiny.
 * 
 * 
 * EXPECTED BEHAVIOR AT DIFFERENT GAIN LEVELS:
 * =============================================
 * 
 * P = 0.0010 (too low)
 *   - Slow ramp to target
 *   - Output stays below 50%
 *   - Takes 2+ seconds to reach target
 *   - Fix: Increase P or D
 * 
 * P = 0.0016869 (current - baseline)
 *   - Moderate ramp
 *   - Output reaches 70-90%
 *   - Takes ~1.5 seconds to reach target
 * 
 * P = 0.0025 (aggressive)
 *   - Fast ramp, output hits 100% quickly
 *   - Takes ~0.5 seconds to reach target
 *   - May oscillate/ring if D too low
 *   - Fix: Increase D to smooth it
 * 
 * D = 0.0 (no damping)
 *   - Oscillatory response, RPM overshoots and bounces
 *   - Rings around target
 *   - Fix: Add D = 0.001 or 0.002
 * 
 * D = 0.0012 (current - light damping)
 *   - Slightly smooth response
 *   - Minimal overshoot
 * 
 * D = 0.003 (heavy damping)
 *   - Very smooth, slow response
 *   - Overshoots less but ramp is lazy
 *   - Balance: use 0.0018-0.0025
 * 
 * 
 * QUICK REFERENCE TABLE:
 * ======================
 * 
 * Goal                          | Increase        | Amount              | Note
 * -------------------------------|-----------------|---------------------|-----------------
 * Faster speed-up               | D then P        | D: +0.001, P: +0.0005 | Damping first!
 * Less oscillation/ringing      | D               | +0.001 to 0.002     | Smooth it out
 * Maintain RPM better (no drift)| I               | 0.00001 to 0.0001   | Only if needed
 * More aggressive response      | P               | +0.0003 to 0.0005   | Watch for ringing
 * Smoother overall              | D               | +0.0005 to 0.001    | Best first step
 * 
 * 
 * TROUBLESHOOTING:
 * ================
 * 
 * "Flywheel reaches 2800 RPM but never quite hits 3000"
 *   → Increase P by 0.0002
 *   → Or add small I value (0.00005)
 * 
 * "RPM oscillates wildly, bounces between 2800 and 3200"
 *   → Increase D to 0.002 or 0.003
 *   → Or decrease P slightly
 * 
 * "It takes 3+ seconds to reach target RPM"
 *   → Increase P to 0.002 or higher
 *   → Increase D to 0.0018 (to prevent overshoot)
 * 
 * "Flywheel doesn't maintain RPM under load"
 *   → Add small I gain (0.00001)
 *   → Or increase P slightly (0.0001 increment)
 * 
 * "Everything looks good, what's next?"
 *   → Fine-tune: decrease kShooterRpmTolerance from 50 to 25
 *   → This makes shooter wait closer to exact target before firing
 *   → Better accuracy but requires solid PID tuning first
 * 
 * 
 * TESTING WITH SMARTDASHBOARD:
 * =============================
 * 
 * 1. Connect to robot via USB or Network Tables
 * 2. Navigate to "PID Shooter Testing" tab
 * 3. Available controls:
 *    - "Dashboard Target RPM": Slider to set target (0-6000)
 *    - "Dashboard P/I/D Gain": Sliders to tune live (not saved to code)
 * 4. Monitoring:
 *    - "Flywheel RPM": Current speed (should track target)
 *    - "Flywheel Output %": Actual power to motor
 *    - "RPM Error": Difference from target (0 is perfect)
 *    - "At Target RPM": Boolean, true when within tolerance
 * 
 * 
 * FILE LOCATIONS:
 * ===============
 * Constants:    src/main/java/frc/robot/Constants.java
 *               → Look for: public static final class ShooterConstants
 * Config:       src/main/java/frc/robot/Configs.java
 *               → Look for: closedLoop.pid(P, I, D)
 * Subsystem:    src/main/java/frc/robot/subsystems/Shooter.java
 *               → Look for: m_flywheelPIDController.setSetpoint()
 * Command:      src/main/java/frc/robot/commands/ShooterPIDTestCommand.java
 *               → For testing PID tuning interactively
 */