# PathPlanner Autonomous Routines Guide

This guide explains how to create, configure, and test autonomous routines using PathPlanner for Team 5588's 2026 FRC robot.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Understanding the Auto System](#understanding-the-auto-system)
3. [Creating Your First Path](#creating-your-first-path)
4. [Using Event Markers](#using-event-markers)
5. [Testing in Simulation](#testing-in-simulation)
6. [Common Patterns](#common-patterns)
7. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Required Software
- **PathPlanner GUI**: Download from [pathplanner.dev](https://pathplanner.dev)
- **WPILib 2026**: Latest version with PathPlanner library
- **VS Code** with WPILib extensions

### Project Setup (Already Complete)
Our robot project already has:
- ✅ PathPlanner vendor library (`PathplannerLib-2026.1.2.json`)
- ✅ `DriveSubsystem` configured with `AutoBuilder`
- ✅ `PathPlannerAutoFactory` with command registration
- ✅ `RobotContainer` with autonomous chooser on SmartDashboard

---

## Understanding the Auto System

### Architecture Overview

```
RobotContainer
├── PathPlannerAutoFactory (command registration)
├── SendableChooser (path selection on dashboard)
└── getAutonomousCommand() → returns selected path

PathPlannerAutoFactory
├── registerCommands() → registers NamedCommands for event markers
├── loadPath(String) → loads .path file as Command
└── loadPathSequence() → chains multiple paths

DriveSubsystem
├── AutoBuilder.configure() → tells PathPlanner how to drive
├── getRobotRelativeSpeeds() → feedback for path following
└── driveRobotRelative(ChassisSpeeds) → accepts commands from PathPlanner
```

### Data Flow During Autonomous

```
1. Path file loaded → PathPlanner reads waypoints & event markers
2. PID controller calculated desired ChassisSpeeds → based on current pose vs target
3. DriveSubsystem.driveRobotRelative() called → robot executes movement
4. Event markers triggered → NamedCommands executed (shoot, intake, etc)
5. Odometry updated → pose used for next PID calculation
6. Loop repeats every 20ms
```

---

## Creating Your First Path

### Step 1: Open PathPlanner

1. Launch **PathPlanner GUI**
2. Open your project folder: `c:\Users\zholl\Documents\Reign\2026\rebuilt2026`
3. PathPlanner auto-detects `deploy/pathplanner/` folder

### Step 2: Create a New Path

#### Method A: Simple Drive Forward
1. **File** → **New Path**
2. Name it: `DriveForward`
3. Set the following waypoints:
   - **Start**: (1.5, 7.0) - starting position
   - **End**: (5.0, 7.0) - 3.5 meters forward

#### Method B: Complex Path with Turns
1. **File** → **New Path**
2. Name it: `ShootAndIntake`
3. Add waypoints by clicking on the field:
   - Click point 1: (1.5, 7.0)
   - Click point 2: (3.0, 5.5) - intermediate waypoint
   - Click point 3: (5.0, 3.0) - final position
4. PathPlanner auto-generates bezier curves between points

### Step 3: Configure Path Settings

**In the Path Properties panel:**

| Setting | Value | Notes |
|---------|-------|-------|
| **Max Velocity** | 4.0 m/s | Start conservative, tune up after testing |
| **Max Acceleration** | 3.0 m/s² | Limits jerky movements |
| **Max Angular Velocity** | π rad/s | Rotation speed (same as drive system) |
| **Direction** | Forward | Keep wheels facing direction of travel |
| **Reversed** | False | Only true for backing up |

### Step 4: Save the Path

- PathPlanner auto-saves to: `deploy/pathplanner/paths/DriveForward.path`
- When you deploy code to the robot, this file transfers to the roboRIO

---

## Using Event Markers

Event markers trigger **NamedCommands** at specific points along a path. Think of them as "do this action when the robot reaches here."

### Available Named Commands

Our robot has these registered commands (see `PathPlannerAutoFactory.java`):

```
Shooter Commands:
  • "Spin Up Shooter" → Start flywheel at dashboard RPM
  • "Stop Shooter" → Stop shooter motors
  
Intake Commands:
  • "Run Intake" → Spin intake motor
  • "Stop Intake" → Stop intake motor
  
Belt/Feeder Commands:
  • "Run Belt" → Spin belt motor (feeds game pieces)
  • "Stop Belt" → Stop belt motor
  
Combination Commands:
  • "Intake and Belt" → Run both simultaneously
  • "Shooter and Belt" → Spin shooter, then engage belt
  
Timing Commands:
  • "Wait 0.5 Seconds" → Pause path execution
  • "Wait 1 Second" → Pause path execution
  
Debug Commands:
  • "Print Path Event" → Logs event marker to console
```

### Adding Event Markers to a Path

1. **Open path in PathPlanner**
2. **Click on the path line** at the desired position
3. **Right-click** → "Add Event Marker"
4. **Type the command name** exactly as listed above:
   - Example: `Spin Up Shooter`
   - Example: `Intake and Belt`
5. **Click somewhere else** to confirm

### Example: Shoot Then Intake Path

```
Path: ShootAndIntake
├─ Waypoint 0 (1.5, 7.0) - Start
│   └─ Event Marker @ 0.5s: "Spin Up Shooter"
│       (shooter spools up while driving forward)
│
├─ Waypoint 1 (3.0, 5.5) - Approaching game piece
│   └─ Event Marker @ 2.0s: "Shooter and Belt"
│       (shoot game piece when aligned)
│
└─ Waypoint 2 (5.0, 3.0) - Intake area
    └─ Event Marker @ 3.5s: "Intake and Belt"
        (intake new game piece while at location)
```

### Event Marker Timing

Event markers execute based on:
- **Time along path** (seconds from start)
- Not position on field - use waypoint proximity instead

**Tips:**
- Mark a waypoint slightly before the action point, then add marker at specific time
- "Wait" commands useful for letting subsystems settle
- Multiple markers can execute in quick succession

---

## Testing in Simulation

### Prerequisites

The project already has WPI Sim configured in `build.gradle` and `RobotSimulation.java` handles simulation setup.

### Running Simulation

1. **Build and Run Simulation**
   ```powershell
   cd c:\Users\zholl\Documents\Reign\2026\rebuilt2026
   ./gradlew simulateJava
   ```

2. **WPILib Simulation GUI opens** showing:
   - Virtual field with robot starting position
   - Physics simulation of drive motors
   - Telemetry display

### Testing Your Path

1. **Robot Simulation window:**
   - Click **SmartDashboard** tab
   - Find **"Auto Choices"** dropdown
   - Select your path: `DriveForward`
   - **Click "Autonomous"** button to start

2. **Monitor Execution:**
   - Watch robot follow path on field display
   - Check console for event marker execution
   - Verify subsystems activate at correct times

3. **Adjust if Needed:**
   - If robot overshoots: ↓ Max Velocity
   - If robot undershoots: ↑ Max Velocity  
   - If path is jerky: ↓ Max Acceleration
   - If robot takes wrong route: adjust waypoints

### Viewing Telemetry

In **SmartDashboard** during simulation:
- `Robot Pose X/Y` - Current robot position
- `Robot Heading` - Current rotation (degrees)
- `PathPlanner State` - Desired vs actual position
- Shooter RPM, Belt Speed, Intake Speed - Subsystem states

---

## Common Patterns

### Pattern 1: Simple Mobility Autonomous
**Goal**: Drive to a location and score

**Path Structure:**
```
1. Start position
2. Drive to scoring area (1 waypoint)
3. Event Marker: "Spin Up Shooter"
4. Drive to optimal angle (1 waypoint)
5. Event Marker: "Shooter and Belt"
```

**Create as:** `MobilityStageTwoPart1.path`

### Pattern 2: Multi-Note Intake Sequence
**Goal**: Collect and score multiple game pieces

**Path Structure:**
```
1. Start (shooter spool-up begins)
   └─ Event: "Spin Up Shooter"
2. Approach first game piece
   └─ Event: "Intake and Belt" (collect piece)
3. Drive to score location
   └─ Event: "Shooter and Belt" (score piece)
4. Drive to next game piece
   └─ Event: "Intake and Belt" (collect piece)
5. Drive back to score
   └─ Event: "Shooter and Belt" (score piece)
```

**Create as:** `MultiNoteAuto.path`

### Pattern 3: Timed Wait for Subsystem
**Goal**: Let shooter spool up before starting movement

**Path Structure:**
```
1. Start at (1.5, 7.0)
   └─ Event @ 0.0s: "Spin Up Shooter"
   └─ Event @ 1.0s: "Wait 1 Second"
2. Drive forward to scoring position
   └─ Event @ 1.5s: "Shooter and Belt"
```

**Why:** Gives shooter 1.5 seconds to reach target RPM before scoring

### Pattern 4: Sequential Paths
**For complex routines, chain multiple paths together**

**In RobotContainer.java:**
```java
m_autoChooser.addOption("Two Path Auto", 
    PathPlannerAutoFactory.loadPathSequence(
        Arrays.asList("Path1", "Path2")
    )
);
```

This executes `Path1.path` completely, then immediately runs `Path2.path` with continuous odometry.

---

## File Organization

### Paths Directory Structure
```
deploy/pathplanner/
├── paths/
│   ├── DriveForward.path           ← Simple test path
│   ├── MobilityStageTwoPart1.path  ← Score then intake
│   ├── MultiNoteAuto.path          ← Multi-piece routine
│   └── [Your custom paths]
│
├── autos/
│   └── (Optional: Complex auto definitions)
│
└── navgrid.json                    ← Field grid for PathPlanner GUI
```

### Adding Paths to Autonomous Chooser

**Edit `RobotContainer.java`:**

```java
// In constructor, after m_autoChooser initialization:

// Add available paths
m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
m_autoChooser.addOption("Drive Forward", 
    PathPlannerAutoFactory.loadPath("DriveForward"));
m_autoChooser.addOption("Mobility Stage Two Part 1", 
    PathPlannerAutoFactory.loadPath("MobilityStageTwoPart1"));
m_autoChooser.addOption("Multi Note Auto", 
    PathPlannerAutoFactory.loadPath("MultiNoteAuto"));

// For sequential paths:
m_autoChooser.addOption("Complex Routine", 
    PathPlannerAutoFactory.loadPathSequence(
        Arrays.asList("Path1", "Path2", "Path3")
    )
);

SmartDashboard.putData("Auto Choices", m_autoChooser);
```

---

## Debugging & Tuning

### Common Issues & Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| Robot doesn't move | Path not loaded | Check path filename in chooser, rebuild code |
| Robot moves wrong direction | Reversed kinematics | Check `driveRobotRelative()` sign conventions |
| Events don't trigger | Wrong command name | Verify exact spelling in `PathPlannerAutoFactory` |
| Path overshoots waypoint | Max velocity too high | Reduce in PathPlanner, rebuild, redeploy |
| Shooter doesn't spin up in time | Event too late | Move "Spin Up Shooter" event earlier in path |
| Robot jerks between waypoints | Acceleration too high | Reduce Max Acceleration in PathPlanner |

### Enable Debug Output

**In `PathPlannerAutoFactory.java`**, add logging:

```java
NamedCommands.registerCommand("Run Intake", 
    Commands.runOnce(() -> {
        System.out.println("DEBUG: Intake event triggered!");
        m_intake.setSpeed(Constants.IntakeConstants.kIntakeSpeed);
    })
);
```

**Monitor console output** in SmartDashboard during autonomous.

### Tuning PID Constants

**Current Values** (in `Constants.java`):
```java
public static final double kPTranslation = 5.0;  // Translation following
public static final double kPRotation = 5.0;     // Rotation following
```

**If robot overshoots:** Decrease values (5.0 → 3.0)
**If robot doesn't reach target:** Increase values (5.0 → 7.0)

Rebuild and redeploy to test changes.

---

## Deployment Checklist

Before deploying to the robot:

- [ ] All paths created and saved in `deploy/pathplanner/paths/`
- [ ] Paths added to `RobotContainer.java` autonomous chooser
- [ ] Event marker names match exactly in `PathPlannerAutoFactory.java`
- [ ] Code builds successfully: `./gradlew build -x test`
- [ ] Tested in simulation first
- [ ] Tested with limited drive speed on field
- [ ] Final tuning complete

---

## Advanced Topics

### Custom Event Markers

To add your own NamedCommand:

1. **Define in `PathPlannerAutoFactory.java`:**
   ```java
   NamedCommands.registerCommand("Custom Action",
       Commands.sequence(
           Commands.runOnce(() -> m_shooter.spinUp()),
           Commands.waitSeconds(0.5),
           Commands.runOnce(() -> m_belt.engage())
       )
   );
   ```

2. **Use in path event marker:** `Custom Action`

### Parallel Command Execution

**Run command while following path:**

```java
m_autoChooser.addOption("Path with Parallel",
    PathPlannerAutoFactory.loadPathWithParallel(
        "MyPath",
        Commands.sequence(
            Commands.runOnce(() -> m_intake.setSpeed(0.5)),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> m_intake.setSpeed(0.0))
        )
    )
);
```

### Pose Estimation with Limelight

During autonomous, vision odometry can correct drift:

**Current implementation:** Only uses wheel odometry
**To add vision:** Modify `DriveSubsystem.periodic()` to apply Limelight corrections

---

## Quick Reference

### Command Registration
- Location: `PathPlannerAutoFactory.java` line 46-88
- How to add: Add `NamedCommands.registerCommand()` call
- How to use in path: Add event marker with command name

### Path Files
- Location: `deploy/pathplanner/paths/`
- Created in: PathPlanner GUI
- Deployed via: `./gradlew deploy`

### Autonomous Chooser
- Location: `RobotContainer.java` lines 54-64
- Edit to: Add new paths with `addOption()`
- Displayed on: SmartDashboard "Auto Choices"

### Drive System
- Kinematics: 27"×27" MAXSwerve frame
- Max Speed: 4.8 m/s
- Max Rotation: 2π rad/s
- Default Path Velocity: 4.0 m/s (conservative)

---

## Support & Resources

- **PathPlanner Docs**: https://pathplanner.dev/getting-started.html
- **WPILib Autonomous**: https://docs.wpilib.org/en/latest/docs/software/commandbased/basic-grouping.html
- **Team Repo**: Team 5588 rebuilt2026 project (this one!)

---

**Last Updated:** February 27, 2026  
**PathPlanner Version:** 2026.1.2  
**WPILib Version:** 2026.0.0  
