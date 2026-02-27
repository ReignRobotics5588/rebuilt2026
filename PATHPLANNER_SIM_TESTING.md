# Testing PathPlanner in Simulation

Yes! You can absolutely test PathPlanner autonomous routines in simulation. Here's how:

## Quick Start (5 minutes)

### Step 1: Build and Run Simulation
```powershell
cd c:\Users\zholl\Documents\Reign\2026\rebuilt2026
./gradlew simulateJava
```

**Wait for the WPI Simulation GUI to open** (usually 10-20 seconds)

### Step 2: Open SmartDashboard
1. In the Simulation GUI window, find the **"SmartDashboard"** tab
2. Look for **"Auto Choices"** dropdown menu
3. Select **"Test Drive Forward"** from the list

### Step 3: Start Autonomous
1. Click the **"Autonomous"** button in the Simulation GUI (or press Space)
2. Watch the virtual robot on the field
3. The robot should **drive forward ~3.5 meters** on the blue alliance side

### Step 4: Verify Success
- ✅ Robot moves forward in a straight line
- ✅ No error messages in console
- ✅ Path completes and stops
- ✅ Telemetry shows robot pose changing

---

## What to Observe During Testing

### Robot Movement Display
- **Top-down field view** shows the robot as a triangle
- **Blue line** = planned path from PathPlanner
- **Robot position** updates as it follows the path
- **Target pose** shown as a small square

### Console Output
```
[Command Scheduler] Scheduling: PathPlannerAuto
[PathPlanner] Following path: TestDriveForward
[Drivetrain] Swerve modules driving...
```

### SmartDashboard Telemetry
- `Robot Pose X`: Should increase from ~1.5 to ~5.0 meters
- `Robot Pose Y`: Should stay around 7.0 meters
- `Robot Heading`: Should stay near 0° (facing forward)
- Subsystem speeds show velocity changes

---

## Troubleshooting

### Robot Doesn't Move
**Issue**: Path loaded but robot stationary
- **Check**: Is "Autonomous" button actually pressed?
- **Check**: Console for errors loading path file
- **Fix**: Rebuild project: `./gradlew simulateJava`

### Path File Not Found Error
**Console shows**: `Could not load path: TestDriveForward`
- **Cause**: Path file not in correct location
- **Fix**: Verify `TestDriveForward.path` exists in:
  ```
  src/main/deploy/pathplanner/paths/
  ```
- **Rebuild**: `./gradlew simulateJava` (deploys paths to sim)

### Robot Moves Wrong Direction
**Issue**: Robot goes backward or sideways instead of forward
- **Check**: Your DriveSubsystem coordinate system
- **Check**: Gyro initialization (should start at 0°)
- **Fix**: May need to adjust path or kinematics signs

### Path Overshoots or Undershoots Waypoints
**Issue**: Robot doesn't reach the target position accurately
- **Cause**: Velocity/acceleration too aggressive
- **Fix**: In `TestDriveForward.path`, change:
  ```json
  "globalConstraints": {
    "maxVelocity": 2.0,        // ← Reduce from 3.0
    "maxAcceleration": 1.5     // ← Reduce from 2.0
  }
  ```
- Rebuild and retest

---

## Testing More Complex Paths

### Create a Path with Event Markers

1. **Open PathPlanner GUI**
2. **File** → **Open Project** → Select your project folder
3. **Create New Path** called `TestWithMarker`
4. Add waypoints: (1.5, 7.0) → (5.0, 7.0)
5. **Add Event Marker**:
   - Click on the path line
   - Right-click → Add Event Marker
   - Type: `Print Path Event`
   - Set time: 0.5s
6. **Save** (auto-saves to `deploy/pathplanner/paths/`)

### Add to RobotContainer

Edit `RobotContainer.java`:
```java
m_autoChooser.addOption("Test with Marker", 
    PathPlannerAutoFactory.loadPath("TestWithMarker"));
```

### Test in Simulation
1. Rebuild: `./gradlew simulateJava`
2. Select "Test with Marker" in Auto Choices
3. Press Autonomous
4. **Watch console** - you should see:
   ```
   [Print Path Event] Path event triggered!
   ```

---

## Performance Metrics to Check

| Metric | Expected | Notes |
|--------|----------|-------|
| **Path Following Error** | < 0.2m | How far off the planned path |
| **Angle Error** | < 5° | Rotation accuracy |
| **Completion Time** | ~4-5s | For 3.5m at 3.0 m/s |
| **Motor Commands** | Smooth | No jerky changes in console |
| **Frame Rate** | 50 FPS | Simulation should run smoothly |

---

## Advanced: Parallel Command Testing

Test running subsystem commands while following a path:

**Example** in `RobotContainer.java`:
```java
m_autoChooser.addOption("Path with Intake",
    PathPlannerAutoFactory.loadPathWithParallel(
        "TestDriveForward",
        Commands.sequence(
            Commands.runOnce(() -> m_intake.setSpeed(0.5)),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> m_intake.setSpeed(0.0))
        )
    )
);
```

In simulation, you'll see:
- Robot following path as normal
- Intake speed changing on SmartDashboard at 0s, 2s, 4s

---

## Simulation Tips

### Speed Up Simulation
- If simulation runs slowly, reduce field display update rate
- Set SmartDashboard update interval to 100ms

### Record Telemetry
- AdvantageKit can record sim data (future enhancement)
- For now, use SmartDashboard to monitor values

### Multiple Paths Testing
Create test paths incrementally:
1. `TestDriveForward.path` ← Simple straight line (✅ Working now)
2. `TestTurn.path` ← Add rotation
3. `TestComplex.path` ← Full autonomous routine

Test each independently before combining.

---

## Current Test Path Details

**File**: `TestDriveForward.path`

```json
Waypoints:
  Start:  (1.5, 7.0)  - Blue alliance near wall
  End:    (5.0, 7.0)  - 3.5 meters forward

Constraints:
  Max Velocity:       3.0 m/s
  Max Acceleration:   2.0 m/s²
  Max Angular Vel:    360°/s

Expected Behavior:
  - Robot drives straight forward
  - Takes ~1.2 seconds to complete
  - Stops at end position
```

---

## Quick Command Reference

```bash
# Build and run simulation
./gradlew simulateJava

# Clean build (if having issues)
./gradlew clean simulateJava

# Just build (don't run sim)
./gradlew build -x test

# Run specific test path
# (Select from dashboard during sim)
```

---

## Success Criteria for Testing

✅ **All of these should work:**
- [ ] Simulation GUI opens without crashes
- [ ] Robot appears on field display
- [ ] "Test Drive Forward" loads from dropdown
- [ ] Autonomous button starts the path
- [ ] Robot moves forward steadily
- [ ] Path completes and robot stops
- [ ] SmartDashboard telemetry updates
- [ ] No red error messages in console

**If all green**: Your PathPlanner integration is working! 🎉

---

## Next Steps

1. **Test in Simulation** ← You are here
2. **Create More Complex Paths** using PathPlanner GUI
3. **Add Event Markers** for shooter/intake actions
4. **Deploy to RoboRIO** and test on actual field

---

**Last Updated**: February 27, 2026
