# Vision System Implementation Plan - REBUILT 2026

## Overview

This plan outlines the steps to implement a complete vision system for the 2026 FRC game REBUILT using PhotonVision on OrangePi with multiple cameras.

### Camera Configuration (Team 245)
| Camera | Purpose | Location | Priority |
|--------|---------|----------|----------|
| Left Odometry | AprilTag odometry | Front-left (on swerve module) | P0 - Initial |
| Right Odometry | AprilTag odometry | Front-right (on swerve module) | P0 - Initial |
| Turret Camera | Hub tracking | On turret | P0 - Initial |
| HP Station Camera | Human player alignment | TBD | P1 - Initial |

**Note:** Starting with 2 front-mounted odometry cameras; can expand to back cameras later if needed.

### Hardware Purchased
| Component | Model | Key Specs |
|-----------|-------|-----------|
| Coprocessor | [Orange Pi 5 Pro](https://www.amazon.com/dp/B0DP78CMV5) | RK3588S, 16GB RAM, supports SD/eMMC/NVMe |
| Odometry Cameras (x2) | [Arducam OV9281](https://www.amazon.com/dp/B096M5DKY6) | 720P, **Global Shutter**, 100fps, 70deg HFOV, Monochrome |
| Turret Camera | Microsoft LifeCam HD-3000 | 720P, Rolling shutter, ~68.5deg diagonal FOV |

**Storage Recommendation:** Start with SD card for initial setup (PhotonVision's recommended approach). Can migrate to eMMC later for improved reliability. 

Sources: [PhotonVision Orange Pi Install](https://docs.photonvision.org/en/v2025.0.0-beta-1/docs/installation/sw_install/orange-pi.html), [Arducam OV9281 on AndyMark](https://andymark.com/products/arducam-camera)

### 2026 REBUILT AprilTag Layout

All 32 AprilTags are 8.125" (20.64cm) square, using the tag36h11 family.

#### Red Alliance Tags

| Field Element | Tag IDs | Height | Location |
|---------------|---------|--------|----------|
| **HUB** | 2, 3, 4, 5, 8, 9, 10, 11 | 44.2" (1.12m) | Center of Red alliance side, surrounding scoring area |
| **TRENCH** | 1, 7 | 35.0" (0.89m) | Near Red alliance wall, facing Neutral zone |
| **TRENCH** | 6, 12 | 35.0" (0.89m) | Near Red alliance wall, facing Alliance zone |
| **TOWER WALL** | 13, 14 | 21.75" (0.55m) | Red driver station wall, upper section |
| **TOWER WALL** | 15, 16 | 21.75" (0.55m) | Red driver station wall, center section |

#### Blue Alliance Tags

| Field Element | Tag IDs | Height | Location |
|---------------|---------|--------|----------|
| **HUB** | 18, 19, 20, 21, 24, 25, 26, 27 | 44.2" (1.12m) | Center of Blue alliance side, surrounding scoring area |
| **TRENCH** | 17, 23 | 35.0" (0.89m) | Near Blue alliance wall, facing Neutral zone |
| **TRENCH** | 22, 28 | 35.0" (0.89m) | Near Blue alliance wall, facing Alliance zone |
| **TOWER WALL** | 29, 30 | 21.75" (0.55m) | Blue driver station wall, lower section |
| **TOWER WALL** | 31, 32 | 21.75" (0.55m) | Blue driver station wall, center section |

#### Tag Priority for Vision
1. **HUB Tags (Primary)** - Most important for turret tracking and scoring alignment
   - Red: 2, 3, 4, 5, 8, 9, 10, 11
   - Blue: 18, 19, 20, 21, 24, 25, 26, 27
2. **TOWER WALL Tags** - Best for pose estimation (always visible from most positions)
   - Red: 13, 14, 15, 16
   - Blue: 29, 30, 31, 32
3. **TRENCH Tags** - Useful for field position when near trenches
   - Red: 1, 6, 7, 12
   - Blue: 17, 22, 23, 28

**References:**
- [2026 AprilTag User Guide (PDF)](https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-apriltag-images-user-guide.pdf)
- [WPILib Field Layout JSON](https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-welded.json)

---
#### Assign week numbers or dates to these phases so that you can plan to get everything completed before the robot is ready
---

## Phase 1: Pre-Robot Software Setup

### 1.1 PhotonVision OrangePi Setup
**Owner: Vision Subteam**

- [ ] Flash OrangePi with the latest PhotonVision image
- [ ] Configure static IP address: `10.2.45.11` (Team 245)
- [ ] Connect to robot network and verify web dashboard at `http://10.2.45.11:5800`
- [ ] Ensure PhotonVision has been updated in the code to latest version 

### 1.2 Camera Hardware Preparation
**Owner: Vision Subteam + Mechanical**

- [ ] Inspect Arducam OV9281 cameras (x2) and Microsoft LifeCam
- [ ] Test each camera individually on Orange Pi 5 Pro
- [ ] Verify USB bandwidth with multiple cameras connected simultaneously
- [ ] Label cameras clearly: `left_odom`, `right_odom`, `turret`, `hp_station`
- [ ] Note: Arducam OV9281 has adjustable M12 lens - focus at ~2-3m for optimal AprilTag detection

### 1.3 PhotonVision Pipeline Configuration
**Owner: Vision Subteam**

- [ ] Create AprilTag pipeline for `left_odom` camera
- [ ] Create AprilTag pipeline for `right_odom` camera
- [ ] Create hub tracking pipeline for `turret` camera (AprilTag or reflective tape)
- [ ] Create HP station alignment pipeline for `hp_station` camera
- [ ] Configure 3D mode for pose estimation (odometry cameras)
- [ ] Set target family to `tag36h11` (FRC 2026 standard)
- [ ] Download and install [2026 field layout JSON](https://github.com/PhotonVision/photonvision/tree/master/photon-lib/src/main/native/resources/apriltag)
- [ ] Test detection with printed AprilTags at various distances (2m, 4m, 6m)

### 1.4 Simulation Environment Setup
**Owner: Vision Subteam**

- [ ] Create a new branch in Adambots2026 repo called subsystem/vision
- [ ] Verify PhotonVision vendordep is current (already installed: v2026.2.1)
- [ ] Set up PhotonVision simulation in `simulationPeriodic()`
- [ ] Create simulated cameras matching planned robot configuration
- [ ] Test pose estimation in simulation with AdvantageScope

---

## Phase 1.5: Chassisbot Validation (Before New Robot Ready)

> **Purpose:** Validate the AdambotsLib vision changes work correctly using the chassisbot. This allows early testing before the 2026 robot is built.

### 1.5.1 Chassisbot PhotonVision Update
**Owner: Vision Subteam**

- [ ] Update PhotonVision on chassisbot coprocessor to latest version
- [ ] Verify cameras are recognized after update (`left_odom`, `right_odom`)
- [ ] Re-import 2026 field layout JSON if needed
- [ ] Verify pipelines still work after update

### 1.5.2 Chassisbot Code Setup
**Owner: Vision and Drivetrain Subteams**

- [ ] Measure and document chassisbot camera positions (X, Y, Z from robot center)
- [ ] Update `configureVision()` with chassisbot camera positions:
  ```java
  VisionSystemConfig visionConfig = VisionConfigBuilder.create()
      .addCamera("left_odom")
          .position(Meters.of(X), Meters.of(Y), Meters.of(Z))  // Chassisbot measurements
          .rotation(Degrees.of(0), Degrees.of(PITCH), Degrees.of(0))
          .purpose(CameraPurpose.ODOMETRY)
          .maxTagDistance(Meters.of(4.0))
          .done()
      .addCamera("right_odom")
          .position(Meters.of(X), Meters.of(-Y), Meters.of(Z))
          .rotation(Degrees.of(0), Degrees.of(PITCH), Degrees.of(0))
          .purpose(CameraPurpose.ODOMETRY)
          .maxTagDistance(Meters.of(4.0))
          .done()
      .ambiguityThreshold(0.2)
      .build();
  ```
- [ ] Enable vision code in `configureVision()`
- [ ] Deploy to chassisbot and verify no errors

### 1.5.3 Chassisbot Validation Testing
**Owner: Vision + Drivetrain Subteams**

- [ ] Set up AprilTags in practice area (print tags or use existing field elements)
- [ ] Verify cameras detect tags in PhotonVision dashboard
- [ ] Test pose estimation accuracy:
  - [ ] Place robot at known position, compare to estimated pose
  - [ ] Test at multiple distances (1m, 2m, 3m, 4m)
  - [ ] Verify `maxTagDistance` filtering works (tags beyond limit are ignored)
- [ ] Test pose estimation while driving:
  - [ ] Drive in straight line, verify pose updates smoothly
  - [ ] Drive in circles, verify rotation tracking
  - [ ] Test rapid direction changes
- [ ] Verify AdvantageScope logging shows vision data correctly
- [ ] Test `Utils.isOnRedAlliance()` returns correct value when connected to Driver Station
- [ ] Document any issues or tuning needed for AdambotsLib

---

## Phase 2: Code Implementation

### 2.1 Configure Vision in RobotContainer
**File:** `src/main/java/com/adambots/RobotContainer.java`

- [ ] Add `private PhotonVision vision;` field to RobotContainer
- [ ] Update `configureVision()` with the new 3-step pattern:

**Step 1: Build the configuration**
- Uncomment and update the configureVision

**Step 2: Create PhotonVision**
- Create a PhotonVision object

**Step 3: Connect vision to swerve**
- Set it in Swerve

### 2.2 Configure Turret Camera for Tracking
**File:** `src/main/java/com/adambots/RobotContainer.java`

> **Note:** Do NOT create a separate VisionSubsystem. AdambotsLib provides all vision functionality via the `vision` field you created above.

- [ ] Access turret tracking via the `vision` field:
  ```java
  // Get best target from turret camera
  Optional<VisionTarget> target = vision.getBestTargetFromCamera("turret");

  // Get yaw to specific HUB tag
  Rotation2d yaw = vision.getYawToAprilTag(hubTagID);

  // Check if any HUB tag is visible
  int visibleHub = vision.hasID(Constants.VisionConstants.getHubTags(alliance));
  ```

### 2.3 Integrate Turret Tracking Commands (work with Shooter Subteam)
**File:** `src/main/java/com/adambots/commands/TurretTrackingCommands.java`

- [ ] Wire vision suppliers using the `vision` field:
  ```java
  // In RobotContainer button bindings:
  DoubleSupplier targetAngle = () -> {
      Optional<VisionTarget> target = vision.getBestTargetFromCamera("turret");
      return target.map(VisionTarget::getYaw).orElse(0.0);
  };
  BooleanSupplier hasTarget = () ->
      vision.getBestTargetFromCamera("turret").isPresent();

  Buttons.XboxLeftBumper.whileTrue(
      TurretTrackingCommands.trackTargetCommand(shooter, targetAngle, hasTarget)
  );
  ```
- [ ] Test `trackTargetCommand()` in simulation
- [ ] Configure tracking PID constants in Constants.java

### 2.4 Add Vision Telemetry
**Owner: Vision Subteam**

**Automatic pose estimation:**
- `updatePoseEstimation()` is called automatically by AdambotsLib's `SwerveSubsystem.periodic()` after `setupVision()` is called
- No additional code needed for pose updates

**Optional Field2d visualization:**
`updateVisionField()` is also called automatically by SwerveSubsystem when vision is connected.

**Dashboard telemetry (optional):**
Add to `configureDashboard()` for driver visibility:
```java
SmartDashboard.putBoolean("Vision/HasTarget", vision.hasTarget());
SmartDashboard.putString("Vision/DetectedTags", vision.getAllDetectedTagIds().toString());
SmartDashboard.putNumber("Vision/ClosestTagDistance", vision.getDistanceToClosestTag());
```

- [ ] Verify pose updates appear in AdvantageScope after `setupVision()` is called
- [ ] (Optional) Add dashboard telemetry for driver visibility

---

## Phase 3: Robot Integration (When Robot Ready)

### 3.1 Physical Camera Mounting
**Owner: Vision with Mechanical & Electrical Subteams**

- [ ] Design camera mounts for swerve modules (coordinate with CAD)
- [ ] Mount `left_odom` camera on front-left swerve module with clear forward view
- [ ] Mount `right_odom` camera on front-right swerve module with clear forward view
- [ ] Mount `turret` camera (LifeCam) on turret assembly, centered on rotation axis
- [ ] Ensure cameras are rigid and won't vibrate during movement
- [ ] Provide CAD measurements to programming team (X, Y, Z from robot center, pitch angle)

### 3.2 Camera Calibration
**Owner: Vision Subteam**

- [ ] Print calibration checkerboard (8x8, 1" squares) on rigid board
- [ ] Calibrate `left_odom` Arducam in PhotonVision (capture 12+ images at various angles)
- [ ] Calibrate `right_odom` Arducam in PhotonVision
- [ ] Calibrate `turret` LifeCam in PhotonVision
- [ ] Calibrate `hp_station` camera in PhotonVision (when added)
- [ ] Export and save calibration files to `deploy/vision/` folder
- [ ] Verify calibration: measured distance to tag should match PhotonVision reported distance (+/-5%)

### 3.3 Camera Position Calibration
**Owner: Vision Subteam**

- [ ] Measure camera positions from robot center (X, Y, Z)
- [ ] Measure camera angles (roll, pitch, yaw)
- [ ] Update `configureVision()` with measured values
- [ ] Fine-tune using known AprilTag positions

### 3.4 Field Testing
**Owner: All Subteams**

- [ ] Set up practice field with AprilTags at known positions
- [ ] Test pose estimation accuracy at various distances
- [ ] Test pose estimation while moving
- [ ] Verify multi-tag detection improves accuracy
- [ ] Test turret tracking responsiveness
- [ ] Tune standard deviation values based on observed accuracy

---

## Phase 4: Advanced Features 

### 4.1 Human Player Station Alignment
**Owner: Vision + Programming Subteams**

- [ ] Determine optimal HP camera mounting location
- [ ] Mount HP station camera with view of HP station AprilTags
- [ ] Create alignment pipeline in PhotonVision (may use colored targets or AprilTags)
- [ ] Implement `alignToHPStationCommand()` command
- [ ] Add driver feedback (LEDs show alignment status, dashboard indicator)
- [ ] Test alignment accuracy and driver usability

### 4.2 Automatic Pose Reset
- [ ] Implement pose reset when seeing known tags at match start
- [ ] Add autonomous initialization sequence
- [ ] Test with different starting positions

---

## Dependencies & Resources

### Required Downloads
- [PhotonVision OrangePi Image](https://docs.photonvision.org)
- [2026 Field Layout JSON](https://github.com/PhotonVision/photonvision/tree/master/photon-lib/src/main/native/resources/apriltag)
- [2026 AprilTag Images](https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-apriltag-images-user-guide.pdf)

### Reference Documentation
- [PhotonVision Docs](https://docs.photonvision.org)
- [WPILib Vision](https://docs.wpilib.org/en/stable/docs/software/vision-processing/index.html)
- [YAGSL Vision Integration](https://yagsl.gitbook.io/yagsl)
- [AdambotsLib Vision Docs](https://github.com/Adambots-245/AdambotsLib/tree/main/docs/vision)

---
