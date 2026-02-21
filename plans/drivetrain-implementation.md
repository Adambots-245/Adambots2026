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

### 1.1 YAGSL Configuration Review - *Done*
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/swerve/`

- [X] Review existing YAGSL JSON configuration files
- [X] Understand module layout (frontleft, frontright, backleft, backright)
- [X] Review `swervedrive.json` for overall drive configuration
- [X] Review `physicalproperties.json` for wheel diameter, gear ratios
- [X] Review `pidfproperties.json` for PID tuning values
- [X] Review `controllerproperties.json` for heading correction settings

### 1.2 Basic Driving Practice - *Done*
**Owner: Drivetrain Subteam**

- [X] Deploy code to chassisbot
- [X] Test field-centric driving in all directions
- [X] Verify gyro heading resets correctly
- [X] Practice quick direction changes and rotations
- [X] Document any drift or alignment issues
   - Piece hanging off of the robot causing drift (remove the piece)

### 1.3 Telemetry Verification - *Week 4*
**Owner: Drivetrain Subteam**
**File:** `RobotContainer.java`

- [X] Verify module states appear in AdvantageScope
- [X] Verify odometry position updates correctly
- [X] Check wheel velocities match expected values
- [X] Verify turn motor angles are accurate
- [X] Test pose reset functionality

---

## Phase 2: PathPlanner Setup

### 2.1 PathPlanner Configuration - *Done*
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/pathplanner/settings.json`

- [X] Open PathPlanner GUI and connect to project
- [X] Configure robot dimensions (length, width) in settings
- [X] Set up default constraints (max velocity, max acceleration, max angular velocity)
- [X] Verify PathPlanner GUI connects to robot via NetworkTables

### 2.2 Create Test Paths - *Done*
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/pathplanner/paths/`

- [X] Create simple straight-line test path (2m forward)
- [X] Create 90-degree turn path
- [X] Create S-curve path to test smooth motion
- [X] Test each path on chassisbot
- [X] Document any path-following issues

### 2.3 Named Commands Setup - *Done*
**Owner: Drivetrain Subteam**
**File:** `RobotContainer.java`

- [X] Register placeholder named commands for subsystem actions
- [X] Create test autonomous routine combining paths and commands
- [X] Test command sequencing during path execution
- [X] Verify event markers trigger correctly

---

## Phase 3: Autonomous Routine Development (On Chassisbot)

### 3.1 Develop Auto Strategy - *Done*
**Owner: Drivetrain + Strategy Subteams**

- [X] Study 2026 REBUILT game manual for scoring opportunities
- [X] Identify key autonomous starting positions
- [X] Plan 1-piece, 2-piece, and 3-piece auto routines
- [X] Consider alliance partner coordination
- [X] Create path sketches for each routine

### 3.2 Build Practice Auto Routines - *Week 5*
**Owner: Drivetrain Subteam**
**Files:** `src/main/deploy/pathplanner/autos/`

- [ ] Create 1-piece auto (simple score and leave)
- [ ] Create multi-piece auto framework (paths only, no real subsystems)
- [ ] Test path accuracy with tape markers on floor
- [ ] Tune path-following PID as needed
- [ ] Document timing of each routine

### 3.3 Auto Selector Setup - *Week 5*
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
