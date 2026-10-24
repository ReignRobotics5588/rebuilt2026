# Flywheel & Drive Tuning Guide (YAMS / SysId)

## Units changed with the YAMS migration

YAMS configures the SPARK encoders so the closed loop works in **mechanism rotations per second (RPS)**, not RPM.

| Gain | Units | Where |
|------|-------|-------|
| `kFlywheelP`, `kFlywheelD` | duty cycle per RPS of error | `Constants.ShooterConstants` |
| `kFlywheelFF` (kV) | **volts** per RPS | `Constants.ShooterConstants` |
| Drive / angle PID | YAGSL units | `src/main/deploy/swerve/modules/pidfproperties.json` |

Gains tuned in RPM (the Bonney Lake code) must be multiplied by 60 to keep the same behaviour.
YAGSL computes the drive kV automatically from the gear ratio when `drive.v` is 0.

## Step 1: Characterize with SysId (Test mode, operator controller)

Data is logged by AdvantageKit. Open the robot's `.wpilog` in the WPILib SysId tool afterwards.

| Hold | POV up | POV down | POV right | POV left |
|------|--------|----------|-----------|----------|
| nothing (drive) | quasistatic fwd | quasistatic rev | dynamic fwd | dynamic rev |
| left bumper (flywheel) | quasistatic fwd | quasistatic rev | dynamic fwd | dynamic rev |

- Drive routines drive the robot **straight forward/backward**. Give it several meters of open floor and be ready to release.
- Run all four routines for a mechanism, holding each until it has run a few seconds (or the robot runs out of room).
- In SysId, use the `Drive/SysIdState` or `Shooter/SysIdState` entry as the test state. Use `Drive/DrivePositionsMeters`, `DriveVelocitiesMetersPerSec` and `DriveAppliedVolts` (one module is enough), or `Shooter/FlywheelPositionRotations` and `FlywheelAppliedVolts`.

## Step 2: Apply the results

- **Flywheel:** put SysId's kV (V/(rot/s)) into `kFlywheelFF`, and kS too if it's significant. Then raise `kFlywheelP` until the flywheel recovers quickly after a shot without oscillating.
- **Drive:** put kS/kV/kA into `pidfproperties.json` → `drive.s/v/a` (YAGSL then stops auto-computing kV), then tune `drive.p`.

## Step 3: Fill the shot table

`ShooterConstants.kDistanceToRPM` maps hub distance (meters, robot center to hub center) to flywheel RPM.
The starting points are placeholders.

1. Drive to a spot and read `Shooter/Hub Distance (m)` on the dashboard.
2. Adjust RPM until shots score consistently, and record (distance, RPM).
3. Repeat across the shooting range (4 or more points). Values beyond the table clamp to the nearest end.
