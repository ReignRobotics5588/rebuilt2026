# Limelight vs QuestNav: Comprehensive Comparison & QuestNav Programming Guide

## Table of Contents
1. [Overview](#overview)
2. [Limelight Tracking](#limelight-tracking)
3. [QuestNav Tracking](#questnav-tracking)
4. [Key Differences](#key-differences)
5. [QuestNav Programming Guide](#questnav-programming-guide)
6. [Migration from Limelight to QuestNav](#migration-from-limelight-to-questnav)
7. [Common Pitfalls & Solutions](#common-pitfalls--solutions)

---

## Overview

Both **Limelight** and **QuestNav** are vision-based localization systems for FRC robots, but they use fundamentally different technologies:

| Aspect | Limelight | QuestNav |
|--------|-----------|----------|
| **Primary Tech** | Camera + AprilTag Detection | VIO (Visual-Inertial Odometry) + AprilTag Fusion |
| **Hardware** | USB Camera mounted on robot | Meta Quest 3/3S headset (off-robot) |
| **Localization Type** | Relative (needs visible targets) | Absolute (full-field tracking) |
| **Update Rate** | 90 Hz typical | 120 Hz typical |
| **Accuracy** | ±5-10 cm typical | ±1-2 cm typical |
| **Field Knowledge** | None - must see AprilTags | Full field mapping via VIO |
| **Initialization** | Works immediately | Requires field mapping phase |
| **Communication** | USB or Ethernet | Ethernet (NetworkTables 4) |

---

## Limelight Tracking

### How Limelight Works

Limelight is a **relative pose estimation system** that works by:

1. **Image Capture**: Onboard camera continuously captures images of the field
2. **Target Detection**: Runs AprilTag detection on captured images
3. **Pose Calculation**: Computes robot position relative to detected AprilTag
4. **NetworkTables Output**: Publishes results via NetworkTables

```
┌─────────────────────────────────────────────────┐
│           LIMELIGHT WORKFLOW                     │
├─────────────────────────────────────────────────┤
│                                                   │
│  Camera Sees AprilTag → Detect Tag ID & Location │
│         ↓                                         │
│  Calculate Position Relative to Tag              │
│         ↓                                         │
│  Robot Knows: "I'm 1.5m from Tag 42"            │
│         ↓                                         │
│  Must Match Tag to Field Location                │
│         ↓                                         │
│  Final Pose: (X, Y, Rotation) on Field          │
│                                                   │
└─────────────────────────────────────────────────┘
```

### Limelight Limitations

1. **Requires Visible Targets**: If no AprilTags are in view, you lose position
2. **Limited Range**: Must be close enough to see and identify tags clearly
3. **Line-of-Sight Dependent**: Obstructed or tilted views degrade accuracy
4. **Dead Reckoning Gaps**: No position when targets aren't visible
5. **Initialization**: Must see at least one AprilTag to establish pose

### Example Limelight Code Pattern

```java
// Traditional Limelight usage pattern
public class LimelightSubsystem extends SubsystemBase {
    private NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private NetworkTable limelightTable = ntInstance.getTable("limelight");
    
    @Override
    public void periodic() {
        double tx = limelightTable.getNumber("tx", 0.0);      // Horizontal offset
        double ty = limelightTable.getNumber("ty", 0.0);      // Vertical offset
        double ta = limelightTable.getNumber("ta", 0.0);      // Target area
        boolean hasTarget = limelightTable.getNumber("tv", 0.0) == 1.0;
        
        if (hasTarget) {
            // Calculate position based on detected tag
            Pose2d tagPose = getFieldPose(tx, ty, ta);
            updateRobotPose(tagPose);
        }
    }
}
```

---

## QuestNav Tracking

### How QuestNav Works

QuestNav is an **absolute pose estimation system** using:

1. **Visual-Inertial Odometry (VIO)**: Tracks visual features across image frames + IMU data
2. **Field Mapping**: Quest learns field layout during initialization
3. **AprilTag Fusion**: Refines VIO estimates when tags are visible
4. **Continuous Tracking**: Maintains pose even without visible targets
5. **NetworkTables 4**: Publishes full-field poses at high frequency

```
┌──────────────────────────────────────────────────┐
│         QUESTNAV WORKFLOW                        │
├──────────────────────────────────────────────────┤
│                                                   │
│  Phase 1: FIELD MAPPING                          │
│  ├─ Quest operator walks field with headset      │
│  ├─ VIO learns visual landmarks & layout         │
│  └─ System maps AprilTag locations               │
│         ↓                                         │
│  Phase 2: ROBOT INITIALIZATION                   │
│  ├─ Place robot at known starting position       │
│  ├─ QuestNav establishes global frame reference  │
│  └─ setPose(knownStartingLocation)               │
│         ↓                                         │
│  Phase 3: CONTINUOUS TRACKING                    │
│  ├─ VIO continuously estimates position change   │
│  ├─ AprilTags refine estimate when visible       │
│  └─ No gaps in tracking - always knows position  │
│         ↓                                         │
│  Result: Absolute pose (X, Y, Rotation) on field│
│                                                   │
└──────────────────────────────────────────────────┘
```

### QuestNav Advantages

1. **Full-Field Coverage**: Works everywhere on field, not just near tags
2. **No Line-of-Sight**: Can track through occlusion (brief blocking)
3. **High Accuracy**: Sub-centimeter positioning
4. **Dead Reckoning**: Maintains position between AprilTag sightings
5. **High Update Rate**: 120 Hz streaming poses
6. **Robust Tracking**: Handles dynamic field conditions

### QuestNav Initialization Requirements

```
PRE-MATCH SETUP:
1. Mount Meta Quest 3/3S on field mounting bracket
2. Network connection established (Ethernet to RoboRIO)
3. Field has been mapped (done once per venue)
4. AprilTags placed and database updated

MATCH START:
1. Robot placed at known starting position
2. Call: questNav.setPose(startingPose)
3. System is ready to track continuously
4. No recalibration needed during match
```

---

## Key Differences

### Fundamental Architecture Difference

**Limelight**: Passive camera looking outward
```
Robot → Limelight (looks for tags) → "I see Tag 42 at this offset"
```

**QuestNav**: Active environmental mapping system
```
Field → Quest (maps environment) → "Robot is at global position X,Y"
```

### Coordinate System Differences

| System | Coordinate Origin | Reference Frame | Updates |
|--------|------------------|-----------------|---------|
| **Limelight** | Relative to visible tag | Target-centric | Only when tags visible |
| **QuestNav** | Global field coordinates | Field-centric | Continuous at 120 Hz |

### Performance Comparison

| Scenario | Limelight | QuestNav |
|----------|-----------|----------|
| Robot can see AprilTag | ✅ 5-10 cm accuracy | ✅✅ 1-2 cm accuracy |
| Robot moves without seeing tags | ❌ Lost position | ✅ Maintains track via VIO |
| Rapid movement | ⚠️ Loses track easily | ✅ Keeps pace with motion |
| Multiple robots on field | ⚠️ Can confuse tag IDs | ✅ Independent absolute positions |
| Lighting conditions | ⚠️ Sensitive to glare | ✅ More robust |

---

## QuestNav Programming Guide

### 1. Basic Setup

#### Step 1: Add Dependency
Create/update `vendordeps/questnavlib.json`:
```json
{
  "fileName": "questnavlib.json",
  "name": "questnavlib",
  "version": "2026-2.2.0",
  "uuid": "a706fe68-86e5-4aed-92c5-ce05aca007f0",
  "frcYear": "2026",
  "mavenUrls": [
    "https://maven.questnav.gg/releases",
    "https://maven.questnav.gg/snapshots"
  ],
  "jsonUrl": "https://maven.questnav.gg/snapshots/gg/questnav/questnavlib-json/2026-2.2.0/questnavlib-json-2026-2.2.0.json",
  "javaDependencies": [
    {
      "groupId": "gg.questnav",
      "artifactId": "questnavlib-java",
      "version": "2026-2.2.0"
    }
  ],
  "cppDependencies": [],
  "jniDependencies": []
}
```

#### Step 2: Create QuestNav Subsystem
```java
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * QuestNav vision subsystem for full-field robot localization.
 * Uses Meta Quest headset with VIO + AprilTag fusion for sub-centimeter accurate tracking.
 */
public class QuestNav extends SubsystemBase {
  // Create instance of QuestNav library
  private final gg.questnav.questnav.QuestNav m_questNav = 
      new gg.questnav.questnav.QuestNav();
  
  // Calibrate this based on how Quest is mounted relative to robot origin
  private static final Transform3d ROBOT_TO_QUEST = new Transform3d();
  
  private Pose2d m_robotPose = new Pose2d();
  private boolean m_hasValidPose = false;
  private boolean m_isTracking = false;

  @Override
  public void periodic() {
    // REQUIRED: Must call this every cycle for QuestNav to function
    m_questNav.commandPeriodic();
    
    updatePoseData();
    updateTelemetry();
  }

  /**
   * Update pose from QuestNav frames
   */
  private void updatePoseData() {
    try {
      // Get all unread pose frames since last call
      Object[] frames = (Object[]) m_questNav.getAllUnreadPoseFrames();
      
      if (frames != null && frames.length > 0) {
        Object latestFrame = frames[frames.length - 1];
        
        // Use reflection to safely access PoseFrame methods
        m_isTracking = (boolean) latestFrame.getClass()
            .getMethod("isTracking").invoke(latestFrame);
        
        if (m_isTracking) {
          Pose3d questPose3d = (Pose3d) latestFrame.getClass()
              .getMethod("questPose3d").invoke(latestFrame);
          
          // Transform from Quest frame to robot frame
          Pose3d robotPose3d = questPose3d.transformBy(ROBOT_TO_QUEST.inverse());
          
          // Convert to 2D
          m_robotPose = new Pose2d(
              robotPose3d.getX(),
              robotPose3d.getY(),
              robotPose3d.getRotation().toRotation2d()
          );
          
          m_hasValidPose = true;
        } else {
          m_hasValidPose = false;
        }
      }
    } catch (Exception e) {
      m_hasValidPose = false;
      SmartDashboard.putString("QuestNav/Error", e.getMessage());
    }
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("QuestNav/X", m_robotPose.getX());
    SmartDashboard.putNumber("QuestNav/Y", m_robotPose.getY());
    SmartDashboard.putNumber("QuestNav/Heading", m_robotPose.getRotation().getDegrees());
    SmartDashboard.putBoolean("QuestNav/Valid", m_hasValidPose);
    SmartDashboard.putBoolean("QuestNav/Tracking", m_isTracking);
  }

  public Pose2d getRobotPose() {
    return m_robotPose;
  }

  public boolean hasValidPose() {
    return m_hasValidPose;
  }

  public boolean isTracking() {
    return m_isTracking;
  }

  /**
   * Set the robot's pose (typically at match start)
   */
  public void setPose(Pose2d newPose) {
    // Call into QuestNav library to reset pose
    // Pattern: m_questNav.setPose(new Pose3d(...));
    m_robotPose = newPose;
  }
}
```

### 2. Integration with Drive Subsystem

```java
public class DriveSubsystem extends SubsystemBase {
    private QuestNav m_questNav;
    private SwerveDriveOdometry m_odometry;
    
    public DriveSubsystem(QuestNav questNav) {
        m_questNav = questNav;
    }
    
    @Override
    public void periodic() {
        // Update odometry with QuestNav pose
        if (m_questNav.hasValidPose()) {
            // Option 1: Use QuestNav as sole source of truth
            m_pose = m_questNav.getRobotPose();
            
            // Option 2: Fuse with wheel odometry (advanced)
            // fusePoseEstimates(m_questNav.getRobotPose(), wheelOdometry);
        }
    }
    
    public Pose2d getPose() {
        return m_pose;
    }
}
```

### 3. Autonomous Commands with QuestNav

```java
/**
 * Drive to a target pose using QuestNav feedback
 */
public class QuestNavAlignCommand extends CommandBase {
    private DriveSubsystem m_drive;
    private QuestNav m_questNav;
    private Pose2d m_targetPose;
    
    private PIDController m_xController = new PIDController(1.0, 0.0, 0.0);
    private PIDController m_yController = new PIDController(1.0, 0.0, 0.0);
    private PIDController m_rotationController = new PIDController(1.0, 0.0, 0.0);
    
    public QuestNavAlignCommand(DriveSubsystem drive, QuestNav questNav, Pose2d target) {
        m_drive = drive;
        m_questNav = questNav;
        m_targetPose = target;
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        if (!m_questNav.hasValidPose()) {
            return; // Can't navigate without position
        }
        
        Pose2d currentPose = m_questNav.getRobotPose();
        
        // Calculate distance and angle to target
        double distanceToTarget = currentPose.getTranslation()
            .getDistance(m_targetPose.getTranslation());
        double angleToTarget = calculateAngleToTarget(currentPose, m_targetPose);
        
        // Compute motor speeds using PID
        double xSpeed = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
        double ySpeed = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
        double rotSpeed = m_rotationController.calculate(
            currentPose.getRotation().getDegrees(),
            m_targetPose.getRotation().getDegrees()
        );
        
        // Command drive
        m_drive.drive(xSpeed, ySpeed, rotSpeed, false);
    }
    
    @Override
    public boolean isFinished() {
        if (!m_questNav.hasValidPose()) return false;
        
        Pose2d current = m_questNav.getRobotPose();
        return current.getTranslation().getDistance(m_targetPose.getTranslation()) < 0.1;
    }
}
```

### 4. Key API Methods

#### Getting Pose Data
```java
// Get current robot pose (2D field coordinates)
Pose2d pose = m_questNav.getRobotPose();
double x = pose.getX();
double y = pose.getY();
double heading = pose.getRotation().getDegrees();

// Check if tracking
if (m_questNav.isTracking()) {
    // QuestNav is actively seeing visual features
}

if (m_questNav.hasValidPose()) {
    // Pose estimate is valid and recent
}
```

#### Setting Pose (Robot Initialization)
```java
// Reset to known starting position at match start
Pose2d startingPose = new Pose2d(1.5, 3.2, Rotation2d.fromDegrees(0));
m_questNav.setPose(startingPose);
```

#### Accessing Raw Frame Data
```java
// Get all unread pose frames since last call
Object[] frames = (Object[]) m_questNav.getAllUnreadPoseFrames();

for (Object frame : frames) {
    // Check if this frame has valid tracking
    boolean tracking = (boolean) frame.getClass()
        .getMethod("isTracking").invoke(frame);
    
    // Get 3D pose from Quest frame
    Pose3d pose3d = (Pose3d) frame.getClass()
        .getMethod("questPose3d").invoke(frame);
    
    // Get timestamp
    long timestamp = (long) frame.getClass()
        .getMethod("dataTimestamp").invoke(frame);
}
```

### 5. Calibration: Robot-to-Quest Transform

The `ROBOT_TO_QUEST` transform is **critical** - it tells the system how the Quest is mounted:

```java
// If Quest is mounted 30cm in front, 0cm left, 10cm up
// with no rotation relative to robot:
private static final Transform3d ROBOT_TO_QUEST = new Transform3d(
    new Translation3d(0.30, 0.0, 0.10),  // XYZ offset in meters
    new Rotation3d()                      // No rotation
);

// If Quest is mounted at an angle (e.g., 45° forward tilt):
private static final Transform3d ROBOT_TO_QUEST = new Transform3d(
    new Translation3d(0.30, 0.0, 0.10),
    new Rotation3d(
        Math.toRadians(45),  // Roll (X axis)
        0,                    // Pitch (Y axis)
        0                     // Yaw (Z axis)
    )
);
```

**Calibration Process:**
1. Place robot at known position (e.g., corner of field)
2. Verify QuestNav reports correct position
3. If off, adjust transform incrementally
4. Repeat at different field positions to validate

---

## Migration from Limelight to QuestNav

### Step-by-Step Migration Checklist

1. **Create QuestNav subsystem** (see section 3.1)
2. **Update RobotContainer**:
   ```java
   // OLD:
   private final Limelight m_limelight = new Limelight();
   
   // NEW:
   private final QuestNav m_questNav = new QuestNav();
   ```

3. **Update Drive subsystem**:
   ```java
   // Change pose source
   public Pose2d getPose() {
       return m_questNav.getRobotPose();  // Was: m_limelight.getRobotPose()
   }
   ```

4. **Update autonomous commands**:
   ```java
   // OLD: new LimelightAlignCommand(drive, limelight, targetPose)
   // NEW: new QuestNavAlignCommand(drive, questNav, targetPose)
   ```

5. **Update initialization code**:
   ```java
   @Override
   public void teleopInit() {
       // OLD: m_limelight.resetPose(startingPose);
       // NEW:
       m_questNav.setPose(startingPose);
   }
   ```

6. **Test thoroughly**:
   - Verify pose tracking across entire field
   - Test autonomous routines
   - Validate accuracy at different positions
   - Check update rate on dashboard

### Compatibility Layer (During Transition)

If you need to support both systems temporarily:

```java
public interface VisionSystem {
    Pose2d getRobotPose();
    boolean hasValidPose();
}

public class LimelightAdapter implements VisionSystem {
    private Limelight m_limelight;
    // ... implementation
}

public class QuestNavAdapter implements VisionSystem {
    private QuestNav m_questNav;
    // ... implementation
}

// In RobotContainer:
private VisionSystem visionSystem = new QuestNavAdapter(m_questNav);
```

---

## Common Pitfalls & Solutions

### ⚠️ Pitfall 1: Forgetting to Call `commandPeriodic()`

**Problem**: QuestNav doesn't update - always reports old pose

**Solution**:
```java
@Override
public void periodic() {
    m_questNav.commandPeriodic();  // MUST be called every cycle
    updatePoseData();
}
```

### ⚠️ Pitfall 2: Incorrect Robot-to-Quest Transform

**Problem**: Reported pose is offset from actual position

**Solution**:
- Measure physical mounting offset carefully
- Verify with known field positions
- Use Shuffleboard to visualize discrepancy
- Calibrate incrementally

### ⚠️ Pitfall 3: Using Pose Without Checking `hasValidPose()`

**Problem**: Code uses zero/stale pose when tracking is lost

**Solution**:
```java
public void periodic() {
    if (m_questNav.hasValidPose()) {
        Pose2d pose = m_questNav.getRobotPose();
        // Use pose
    } else {
        // Use fallback (encoder odometry, last known pose, etc.)
    }
}
```

### ⚠️ Pitfall 4: Field Not Mapped Before Use

**Problem**: "QuestNav can't locate" errors at match start

**Solution**:
1. Ensure field is mapped with Quest at venue
2. Verify AprilTag database is updated
3. Test mapping process before competition
4. Have backup mapping procedure

### ⚠️ Pitfall 5: Pose Not Reset at Match Start

**Problem**: Autonomous drives to wrong location

**Solution**:
```java
@Override
public void autonomousInit() {
    Pose2d startingPose = new Pose2d(1.5, 3.2, Rotation2d.fromDegrees(0));
    m_questNav.setPose(startingPose);
    // Now autonomous can use getRobotPose() accurately
}
```

### ⚠️ Pitfall 6: Not Handling NetworkTables Latency

**Problem**: Autonomous command uses old pose data

**Solution**:
```java
private long m_lastUpdateTime = System.currentTimeMillis();
private static final long UPDATE_TIMEOUT_MS = 50; // Expect update within 50ms

public void periodic() {
    m_questNav.commandPeriodic();
    
    // Check if updates are arriving
    if (System.currentTimeMillis() - m_lastUpdateTime > UPDATE_TIMEOUT_MS) {
        m_hasValidPose = false;  // Mark as stale
        SmartDashboard.putString("QuestNav/Status", "No Recent Updates");
    }
    
    if (m_questNav.isTracking()) {
        m_lastUpdateTime = System.currentTimeMillis();
        m_hasValidPose = true;
    }
}
```

---

## Summary Table: When to Use Each System

| Scenario | Limelight | QuestNav |
|----------|-----------|----------|
| **Budget-conscious system** | ✅ Best | ❌ Expensive |
| **Full-field autonomous** | ❌ Limited | ✅✅ Excellent |
| **Near AprilTags only** | ✅ Good | ✅✅ Excellent |
| **Rapid robot movement** | ❌ Loses track | ✅ Keeps up |
| **Outdoor/sunlit fields** | ⚠️ Glare issues | ✅ More robust |
| **Quick setup at event** | ✅ Minimal calibration | ❌ Needs field mapping |
| **Position accuracy requirement** | ✅ 5-10 cm | ✅✅ 1-2 cm |
| **Independence from field features** | ❌ Needs tags | ✅ Full VIO |

---

## Resources

- **QuestNav Official Docs**: https://questnav.gg/docs
- **QuestNav Robot Code**: https://questnav.gg/docs/getting-started/robot-code
- **FRC Vision Systems**: https://docs.wpilib.org/en/stable/docs/software/vision-processing/

---

**Last Updated**: March 28, 2026  
**Maintained By**: Reign Robotics 5588
