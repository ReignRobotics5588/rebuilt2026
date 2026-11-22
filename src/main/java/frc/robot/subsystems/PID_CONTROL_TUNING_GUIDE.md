# Flywheel PID + Feed-Forward Tuning Guide (Neo Vortex)

## Overview

This guide explains how to tune the shooter flywheel PID controller on a REV Robotics SparkFlex with a **Neo Vortex motor** for consistent RPM control in FRC applications.

### Key Characteristics
- **Motor:** Neo Vortex (6784 RPM max)
- **Controller:** REV SparkFlex with internal closed-loop velocity control
- **Control Method:** PID + Feed-Forward (FF)
- **Target Speed:** 3000 RPM

---

## The Control Architecture

### What is Feed-Forward + PID?

The motor output is calculated as:

```
Motor Output = (FF × Target RPM) + PID(error)
              ↓                   ↓
          Predictive         Corrective
         (anticipates)      (reacts to error)
```

**Feed-Forward (FF):**
- Predicts how much power is needed to reach target RPM
- Gets the motor spinning in the right direction immediately
- Reduces the work PID has to do

**PID:**
- Corrects remaining error after FF does its job
- P: How fast to respond to error
- I: Eliminates systematic drift over time
- D: Dampens oscillation and overshoot

---

## Current Tuning Values

Located in: `src/main/java/frc/robot/Constants.java`

```java
public static final class ShooterConstants {
  // Target speed
  public static final double kShooterTargetRPM = 3000.0;
  public static final double kShooterRpmTolerance = 50.0;
  
  // PID Gains
  public static final double kFlywheelP = 0.0016869;   // Proportional
  public static final double kFlywheelI = 0.0;         // Integral
  public static final double kFlywheelD = 0.0012;      // Derivative
  public static final double kFlywheelFF = 0.000147;   // Feed-Forward (1/6784)
}
```

---

## Step-by-Step Tuning Procedure

### Prerequisites

1. **Robot Setup:**
   - Flywheel motor spinning freely (no load initially)
   - SparkFlex configured for velocity control
   - SmartDashboard connected and running

2. **Dashboard Access:**
   - Shooter telemetry visible in SmartDashboard
   - "PID Shooter Testing/Target RPM" slider available
   - "Shooter/Current RPM" displaying real-time values
   - PID gain sliders accessible: P, I, D

---

### Phase 1: Establish Feed-Forward Baseline

**Objective:** Get the motor spinning to ~80-90% of target RPM with FF alone.

**Theoretical FF for Neo Vortex:**
```
FF = 1 / MaxRPM
FF = 1 / 6784 ≈ 0.000147
```

**Test Procedure:**

1. Set `kFlywheelFF = 0.000147` (if not already set)
2. Set all gains to zero (P=0, I=0, D=0)
3. Set dashboard target RPM to 3000
4. Observe motor behavior:

| Outcome | Status | Action |
|---------|--------|--------|
| Motor reaches ~2700-2900 RPM | ✅ Good | Continue to Phase 2 |
| Motor overshoots to 3500+ RPM | ⚠️ Too high | Decrease FF by 0.00005 |
| Motor only reaches ~2000 RPM | ⚠️ Too low | Increase FF by 0.00005 |
| Motor oscillates wildly | ❌ Bad | Reduce FF significantly |

**Once FF is good, proceed to Phase 2.**

---

### Phase 2: Tune Proportional Gain (P)

**Objective:** P corrects the remaining error after FF gets the motor spinning.

**Why not just increase FF?**
- Higher FF causes overshoot and oscillation
- P responds to error, FF provides baseline
- This separation keeps the system stable

**Test Procedure:**

1. Keep FF constant at 0.000147
2. Set P = 0.001 (start lower than current)
3. Set I = 0, D = 0
4. Set target RPM to 3000 on dashboard
5. Measure the steady-state error:

```
Error = Target RPM - Current RPM
Example: 3000 - 2950 = 50 RPM error
```

6. Adjust P based on error:

| Current Error | Action | New P Value |
|---|---|---|
| Error = 0-50 RPM | ✅ Stop, P is good | Keep current P |
| Error = 50-150 RPM | Increase P | P + 0.0001 |
| Error = 150+ RPM | Increase P more | P + 0.0003 |
| Error oscillates ±50 RPM | Decrease P | P - 0.0001 |

**Continue adjusting P until error is 0-50 RPM consistently.**

**Your current P value (0.0016869) is likely good** - it was tuned to handle ~50 RPM error.

---

### Phase 3: Tune Derivative Gain (D)

**Objective:** D dampens the response, preventing oscillation and overshoot.

**When to add D:**

1. **If P is causing oscillation:** Motor bounces between RPM values
2. **If you increased P above 0.002:** Needs damping
3. **Default is usually good:** 0.5-1.0 × P ratio

**Test Procedure:**

1. Increase P slowly until motor oscillates around target
2. Example: P = 0.003 causes ±100 RPM oscillation
3. Add D to smooth it:

| D Value | Effect |
|---|---|
| D = 0 | Oscillatory, bouncy response |
| D = 0.0005 | Slightly smoother |
| D = 0.001 | Better smoothing |
| D = 0.002 | Good damping (current) |
| D = 0.003+ | Over-damped, sluggish response |

**Rule of thumb:**
```
D ≈ 0.5 to 1.0 × P
Example: P = 0.0016869, D ≈ 0.0008 to 0.0016869
```

**Your current D (0.0012) is reasonable** - slightly lower than P, which is appropriate.

---

### Phase 4: Add Integral Gain (I) - Only If Needed

**Objective:** I eliminates systematic drift under load (game pieces).

**When to add I:**

Only add if the motor drifts down when:
- Shooting game pieces
- Sustained high power demand
- Battery voltage drops significantly

**Do NOT add I if:**
- Motor already maintains RPM with FF+P+D ✓ (Your current state)
- You haven't stabilized P and D yet

**Test Procedure:**

1. Spin flywheel to target (3000 RPM) with NO load
2. Apply load (manually slow the motor, or shoot a game piece)
3. Observe recovery:
   - ✅ **Recovers to target:** I not needed, skip this phase
   - ⚠️ **Drifts down 50+ RPM:** Add small I

**If you need I:**

```java
// Start VERY small
kFlywheelI = 0.00001;

// If still drifting, increase:
kFlywheelI = 0.00002;

// If oscillating after adding I:
kFlywheelI = 0.000005;  // Go back down
```

**WARNING:** Too much I causes oscillation and instability. Increase in 0.00001 increments.

---

## Complete Tuning Workflow

```
START: Set FF baseline (0.000147)
  ↓
TEST: Motor reaches ~80-90% target with FF only
  ├─ YES → Proceed
  └─ NO  → Adjust FF ±0.00005, test again
  ↓
TUNE P: Reduce error to 0-50 RPM
  ↓
TEST: Adjust P until error minimal and stable
  ├─ Oscillates? → Add D
  └─ Still drifts? → Increase P
  ↓
TUNE D: Eliminate oscillation
  ↓
TEST: Smooth ramp to target, no bouncing
  ├─ Sluggish? → Decrease D
  └─ Stable? → Proceed
  ↓
VALIDATE: Real-world shooting test
  ├─ 10+ shoots at 3000 RPM
  ├─ Monitor Current RPM on dashboard
  └─ Check for drift or oscillation
  ↓
ADD I (OPTIONAL): Only if drift under load
  ├─ Drifts? → Add I = 0.00001
  └─ Stable? → DONE
  ↓
SAVE: Copy final values to Constants.java
```

---

## Dashboard Testing

### Access Points

**SmartDashboard Location:**
- Tab: "PID Shooter Testing"
- Sliders: "Target RPM", "P Gain", "I Gain", "D Gain"

**Telemetry Display:**
- Tab: "Shooter"
- Values: Current RPM, Target RPM, P/I/D Gains

### Real-Time Tuning Procedure

1. **Set Target RPM:** Use "Target RPM" slider (0-6000)
2. **Adjust Gains:** Use P/I/D sliders in real-time
3. **Monitor:** Watch "Current RPM" track target
4. **Record:** Note good values, don't save to code yet
5. **Validate:** Run 5-10 test cycles with same values
6. **Copy to Code:** Once stable, update Constants.java

---

## Troubleshooting

### Problem: Motor stuck 250 RPM below target

**Causes:**
- FF too low (not spinning enough)
- P too low (not correcting error)
- Encoder reading wrong RPM values
- Motor current limited

**Solutions:**
1. Increase FF by 0.00005
2. Increase P by 0.0001
3. Verify encoder velocity conversion factor = 1.0
4. Check current limit: `kFlywheelCurrentLimit = 60`

### Problem: Motor oscillates ±50-100 RPM around target

**Causes:**
- P too high (over-aggressive)
- D too low (not dampening)
- FF creates overshoot

**Solutions:**
1. Increase D by 0.0005
2. Decrease P by 0.0001
3. Decrease FF by 0.00005 if overshooting

### Problem: Motor undershoots then oscillates (ringing)

**Causes:**
- P + D not balanced
- High frequency oscillation

**Solutions:**
1. Increase D by 0.001
2. Decrease P by 0.0002
3. Increase FF by 0.00005

### Problem: Motor reaches target but drifts down under load

**Causes:**
- No integral correction
- P insufficient for load

**Solutions:**
1. Add I = 0.00001 (tuning phase 4)
2. Increase P by 0.0001
3. Verify motor isn't hitting current limit

---

## Advanced: Calculate Custom FF from Testing

If theoretical FF (0.000147) isn't working:

**Method 1: Load Test**

```
1. Set all PID gains to 0 (P=0, I=0, D=0)
2. Set target RPM to 2000
3. Measure actual RPM reached: (example: 1600 RPM)
4. Calculate: FF_new = (target / actual) × FF_old
   FF_new = (2000 / 1600) × 0.000147 = 0.000184
5. Test with new FF
```

**Method 2: Max RPM Measurement**

```
1. Run motor at 100% output (no target, just open-loop)
2. Measure actual max RPM: (example: 6500 RPM)
3. Calculate: FF = 1 / measured_max_RPM
   FF = 1 / 6500 = 0.000154
4. Test with new FF
```

---

## Key Constants Reference

### Neo Vortex Motor Specs
- **Max RPM:** 6784
- **Free Speed Current:** ~3A
- **Stall Torque:** 2.1 Nm
- **Weight:** 220g

### Current Configuration
| Parameter | Value | Purpose |
|---|---|---|
| `kFlywheelFF` | 0.000147 | 1/6784 for Neo Vortex |
| `kFlywheelP` | 0.0016869 | Error correction |
| `kFlywheelI` | 0.0 | Load compensation (disabled) |
| `kFlywheelD` | 0.0012 | Oscillation damping |
| `kShooterTargetRPM` | 3000 | Target speed |
| `kShooterRpmTolerance` | 50 | Threshold to engage belt |

---

## Summary

**To tune your shooter:**

1. ✅ **Phase 1:** Verify FF baseline (0.000147)
2. ✅ **Phase 2:** Tune P to reach target (reduce error to 0-50 RPM)
3. ✅ **Phase 3:** Tune D to eliminate oscillation (smooth ramp)
4. ⚠️ **Phase 4:** Add I only if motor drifts under load
5. ✅ **Validate:** Test 10+ shots at full power
6. ✅ **Save:** Copy final values to Constants.java

**Expected Performance:**
- Ramp to 3000 RPM in <1 second
- Smooth, no bouncing
- Hold ±50 RPM under load
- Consistent across multiple cycles

---

## Configuration Files

### Reading Current Values
```
Constants.java
└── ShooterConstants
    ├── kFlywheelP = 0.0016869
    ├── kFlywheelI = 0.0
    ├── kFlywheelD = 0.0012
    └── kFlywheelFF = 0.000147
```

### Modifying Values
1. Edit Constants.java
2. Save and deploy to robot
3. Verify new values on dashboard
4. If unstable, revert and try smaller adjustments

### Dashboard Controls (RobotContainer.periodic())
- Reads P, I, D from dashboard
- Applies changes every robot cycle (20ms)
- Displays current and target RPM in real-time
- No restart needed for gain adjustments

---

## Questions? Key Points to Remember

- **FF is your friend:** Gets the motor spinning efficiently
- **P fixes error:** But not alone (needs D for stability)
- **D prevents bouncing:** Always tune D before increasing P
- **I is optional:** Only add if you have systematic drift
- **Test thoroughly:** 10+ cycles at full power before declaring success
- **Start conservative:** Smaller adjustments prevent instability

Good luck with your tuning! 🚀
