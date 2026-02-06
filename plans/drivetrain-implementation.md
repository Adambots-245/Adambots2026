# Drivetrain Subsystem Implementation Plan - REBUILT 2026

> **Note:** This plan tracks the implementation of the swerve drivetrain, PathPlanner integration, and autonomous routines.

---

## Overview

The drivetrain team is responsible for the swerve drive system and autonomous path execution for the 2026 FRC game REBUILT. This includes:

### Drivetrain Components
- **4 swerve modules** with drive and turn motors
- **4 absolute encoders** (CANcoder) for module angle feedback
- **Pigeon2 gyro** for robot heading
- **YAGSL** for swerve drive control
- **PathPlanner** for autonomous path following

### Hardware Configuration (Team 245)
| Component | Chassisbot (Practice) | New Robot (Competition) |
|-----------|----------------------|------------------------|
| Drive Motors | Kraken X60 (TalonFX) | Kraken X60 (TalonFX) |
| Turn Motors | Neo (SparkMax) | Kraken X44 (TalonFX) |
| Absolute Encoders | CANcoder | CANcoder |
| Gyro | Pigeon2 | Pigeon2 |
| Module Type | MK4i | MK5n |

### Control Strategy
| Mechanism | Control Type | Controller |
|-----------|-------------|------------|
| Drive | Velocity PID + Feed-Forward | YAGSL (on-motor) |
| Turn | Position PID | YAGSL (on-motor) |
| Path Following | Holonomic PID | PathPlanner |

---
#### Assign week numbers or dates to these phases so that you can plan to get everything completed before the robot is ready
---

## Phase 1: Chassisbot Familiarization

### 1.1 YAGSL Configuration Review
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/swerve/`

- [ ] Review existing YAGSL JSON configuration files
- [ ] Understand module layout (frontleft, frontright, backleft, backright)
- [ ] Review `swervedrive.json` for overall drive configuration
- [ ] Review `physicalproperties.json` for wheel diameter, gear ratios
- [ ] Review `pidfproperties.json` for PID tuning values
- [ ] Review `controllerproperties.json` for heading correction settings

### 1.2 Basic Driving Practice
**Owner: Drivetrain Subteam**

- [ ] Deploy code to chassisbot
- [ ] Test field-centric driving in all directions
- [ ] Verify gyro heading resets correctly
- [ ] Practice quick direction changes and rotations
- [ ] Document any drift or alignment issues

### 1.3 Telemetry Verification
**Owner: Drivetrain Subteam**
**File:** `RobotContainer.java`

- [ ] Verify module states appear in AdvantageScope
- [ ] Verify odometry position updates correctly
- [ ] Check wheel velocities match expected values
- [ ] Verify turn motor angles are accurate
- [ ] Test pose reset functionality

---

## Phase 2: PathPlanner Setup

### 2.1 PathPlanner Configuration
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/pathplanner/settings.json`

- [ ] Open PathPlanner GUI and connect to project
- [ ] Configure robot dimensions (length, width) in settings
- [ ] Set up default constraints (max velocity, max acceleration, max angular velocity)
- [ ] Verify PathPlanner GUI connects to robot via NetworkTables

### 2.2 Create Test Paths
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/pathplanner/paths/`

- [ ] Create simple straight-line test path (2m forward)
- [ ] Create 90-degree turn path
- [ ] Create S-curve path to test smooth motion
- [ ] Test each path on chassisbot
- [ ] Document any path-following issues

### 2.3 Named Commands Setup
**Owner: Drivetrain Subteam**
**File:** `RobotContainer.java`

- [ ] Register placeholder named commands for subsystem actions
- [ ] Create test autonomous routine combining paths and commands
- [ ] Test command sequencing during path execution
- [ ] Verify event markers trigger correctly

---

## Phase 3: Autonomous Routine Development (On Chassisbot)

### 3.1 Develop Auto Strategy
**Owner: Drivetrain + Strategy Subteams**

- [ ] Study 2026 REBUILT game manual for scoring opportunities
- [ ] Identify key autonomous starting positions
- [ ] Plan 1-piece, 2-piece, and 3-piece auto routines
- [ ] Consider alliance partner coordination
- [ ] Create path sketches for each routine

### 3.2 Build Practice Auto Routines
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/pathplanner/autos/`

- [ ] Create 1-piece auto (simple score and leave)
- [ ] Create multi-piece auto framework (paths only, no real subsystems)
- [ ] Test path accuracy with tape markers on floor
- [ ] Tune path-following PID as needed
- [ ] Document timing of each routine

### 3.3 Auto Selector Setup
**Owner: Drivetrain Subteam**
**File:** `RobotContainer.java`

- [ ] Implement SendableChooser for auto selection
- [ ] Add multiple auto options to dashboard
- [ ] Test switching between auto routines
- [ ] Verify correct auto runs when selected

---

## Phase 4: New Robot YAGSL Configuration

> **Note:** This phase begins when mechanical/electrical teams provide the new chassis with Kraken X60 + X44 modules.

### 4.1 Hardware Verification
**Owner: Drivetrain + Electrical Subteams**

- [ ] Verify all CAN IDs match configuration
- [ ] Test each drive motor individually (correct direction)
- [ ] Test each turn motor individually (correct direction)
- [ ] Verify CANcoder readings (0° = forward)
- [ ] Check gyro orientation (positive = counter-clockwise)

### 4.2 Update YAGSL Configuration
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/swerve/`

- [ ] Update module JSON files for Kraken X44 turn motors
- [ ] Update `physicalproperties.json` for MK5n modules:
  - New wheel diameter
  - New gear ratios
  - New conversion factors
- [ ] Update motor inversions if needed
- [ ] Configure absolute encoder offsets for each module
- [ ] Test basic module movement before full driving

### 4.3 Initial Drive Test
**Owner: Drivetrain Subteam**

- [ ] Deploy and test basic movement (slow speed first)
- [ ] Verify all modules point the same direction
- [ ] Check for wheel slip or skipping
- [ ] Verify odometry tracks correctly
- [ ] Test gyro heading stability

---

## Phase 5: New Robot Tuning

### 5.1 Drive Motor Tuning
**Owner: Drivetrain Subteam**
**File:** `src/main/deploy/swerve/modules/pidfproperties.json`

- [ ] Characterize drive motors using SysId (optional, if time permits)
- [ ] Tune drive velocity PID (start with low P, add as needed)
- [ ] Tune drive feed-forward (kS, kV, kA)
- [ ] Test velocity tracking at various speeds
- [ ] Document final tuned values

### 5.2 Turn Motor Tuning
**Owner: Drivetrain Subteam**
**File:** `src/main/deploy/swerve/modules/pidfproperties.json`

- [ ] Tune turn position PID (P gain most important)
- [ ] Test module angle accuracy at rest
- [ ] Test module angle accuracy while driving
- [ ] Minimize turn motor oscillation
- [ ] Document final tuned values

### 5.3 Heading Correction Tuning
**Owner: Drivetrain Subteam**
**File:** `src/main/deploy/swerve/controllerproperties.json`

- [ ] Enable heading correction if not already
- [ ] Tune heading PID for straight-line driving
- [ ] Test rotation stability during translation
- [ ] Adjust deadbands as needed
- [ ] Document final tuned values

### 5.4 PathPlanner Re-tuning
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/pathplanner/settings.json`, YAGSL configs

- [ ] Update robot dimensions in PathPlanner settings for new robot
- [ ] Update velocity/acceleration constraints for new robot performance
- [ ] Verify translation/rotation PID in YAGSL configs
- [ ] Test existing paths on new robot
- [ ] Adjust paths as needed for new robot dimensions

---

## Phase 6: Competition Autonomous Routines

### 6.1 Finalize Auto Paths
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/pathplanner/`

- [ ] Update all paths for actual field measurements
- [ ] Integrate real subsystem commands (shooter, intake, etc.)
- [ ] Test complete auto sequences with all subsystems
- [ ] Time each auto routine
- [ ] Create backup/fallback auto routines

### 6.2 Starting Position Calibration
**Owner: Drivetrain Subteam**

- [ ] Create starting position presets for each auto
- [ ] Implement pose reset at auto start
- [ ] Test alignment accuracy from each starting position
- [ ] Create driver alignment guides if needed

### 6.3 Auto Reliability Testing
**Owner: Drivetrain Subteam**

- [ ] Run each auto routine 10+ times
- [ ] Document success rate and failure modes
- [ ] Fix any consistency issues
- [ ] Test with low battery to simulate match conditions
- [ ] Final review with drive team before competition

---

## Phase 7: Advanced Features (Optional)

### 7.1 Vision-Assisted Driving
**Owner: Drivetrain + Vision Subteams**

- [ ] Coordinate with vision team for pose estimation integration
- [ ] Test vision-corrected odometry accuracy
- [ ] Implement auto-align to game elements
- [ ] Test snap-to-angle for scoring positions

### 7.2 Auto Balance / Climb (If Applicable)
**Owner: Drivetrain Subteam**

- [ ] Implement gyro-based balance detection
- [ ] Create auto-balance command if needed
- [ ] Test balance routine reliability
- [ ] Integrate into end-of-match auto sequence

---

## Key Files Reference

| File | Purpose |
|------|---------|
| `src/main/deploy/swerve/swervedrive.json` | Overall swerve configuration |
| `src/main/deploy/swerve/modules/*.json` | Individual module configuration |
| `src/main/deploy/swerve/controllerproperties.json` | Heading correction settings |
| `src/main/deploy/swerve/pidfproperties.json` | PID and feed-forward values |
| `src/main/deploy/pathplanner/settings.json` | Robot dimensions, default constraints |
| `src/main/deploy/pathplanner/paths/` | PathPlanner path files |
| `src/main/deploy/pathplanner/autos/` | PathPlanner auto routine files |
| `RobotContainer.java` | Named commands registration, auto chooser |
| `Constants.java` | Drive constants |

---
