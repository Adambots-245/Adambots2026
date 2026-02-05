# 3D Shooter Simulation Guide

This guide explains how to use the FUEL shooter simulation with AdvantageScope for 3D visualization and testing.

## Overview

The simulation system provides:
- **PhotonVision simulation** - Simulated AprilTag detection for distance-based shooting
- **Projectile physics** - Realistic ball trajectories using maple-sim
- **3D visualization** - AdvantageScope integration for trajectory display
- **Distance-to-RPS tuning** - Interpolating lookup table for shot calibration

## Quick Start

### 1. Run the Simulation

```bash
./gradlew simulateJava
```

### 2. Connect AdvantageScope

1. Open [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope/releases)
2. Click **File > Connect to Simulator**
3. Or manually connect to `localhost:5810`

### 3. Configure 3D View

1. Add a **3D Field** tab
2. Configure the following poses:
   - **Robot**: `SmartDashboard/Field/Robot` or your swerve pose
   - **Trajectories**:
     - `Shooter/SuccessfulShot` (green recommended)
     - `Shooter/MissedShot` (red recommended)

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    AdvantageScope                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐ │
│  │ 3D Field    │  │ Robot Model │  │ FUEL Trajectories│ │
│  │ (2026)      │  │ (GLB)       │  │ (Pose3d array)  │ │
│  └─────────────┘  └─────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────┘
                          ▲
                          │ NetworkTables
┌─────────────────────────────────────────────────────────┐
│                   Robot Simulation                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐ │
│  │ Drivetrain  │  │ VisionSim   │  │ FuelProjectile  │ │
│  │ (YAGSL)     │  │ (AprilTags) │  │ (maple-sim)     │ │
│  └─────────────┘  └─────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

## Using the Simulation

### Triggering a Shot

To launch a projectile from code (e.g., in a command):

```java
import com.adambots.simulation.FuelProjectile;

// Get current state
Pose2d robotPose = swerve.getPose();
double distance = visionSim.getDistanceToTarget();
double rps = shooter.getRPSForDistance(distance);
double exitVelocity = FuelProjectile.calculateExitVelocity(rps);

// Launch the projectile
FuelProjectile.launch(robotPose, exitVelocity);

// Or with moving robot compensation:
FuelProjectile.launch(robotPose, swerve.getChassisSpeeds(), exitVelocity);
```

### Vision-Based Auto-Aiming

The `VisionSimSubsystem` provides simulated AprilTag detection:

```java
// Get distance to target (in meters)
double distance = visionSim.getDistanceToTarget();

// Check if target is visible
if (visionSim.hasTarget()) {
    int tagId = visionSim.getTargetId();
    // Use distance for shooter speed calculation
}
```

### Tuning the Distance-to-RPS Table

The `ShooterSubsystem` contains an interpolating lookup table that maps distances to required flywheel speeds:

```java
// In ShooterSubsystem.java
private static final InterpolatingDoubleTreeMap distanceToRPS = new InterpolatingDoubleTreeMap();
static {
    distanceToRPS.put(1.5, 40.0);   // Close shot
    distanceToRPS.put(2.0, 45.0);
    distanceToRPS.put(2.5, 50.0);
    distanceToRPS.put(3.0, 55.0);   // Mid shot
    distanceToRPS.put(4.0, 62.0);
    distanceToRPS.put(5.0, 70.0);   // Far shot
    distanceToRPS.put(6.0, 78.0);
    distanceToRPS.put(7.0, 85.0);   // Maximum range
}
```

**Tuning Process:**
1. Position the robot at a known distance in simulation
2. Fire shots and observe trajectories in AdvantageScope
3. Adjust RPS values until shots consistently hit the target
4. Repeat at multiple distances

## Adding a Custom Robot Model

### Option 1: Use Team 6328's CAD (Recommended for Testing)

Team 6328 (Mechanical Advantage) has shared shooter CAD that's relevant for 2026 FUEL:

- **2020 Shooter Prototype CAD**: [Onshape Link](https://cad.onshape.com/documents/283bff0bc0d3a95823a49284/w/38d23e3dbe480c0b5f483e0c/e/b360d7a16b35edce363088f6)
  - This shooter design is relevant because 2026 FUEL is similar to 2020 game pieces

- **6328's 2026 Build Thread**: [Chief Delphi](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595)
  - Check here for the latest CAD releases (26A Alpha and 26B Competition robots)

- **6328's 2026 Public Code**: [GitHub](https://github.com/Mechanical-Advantage/RobotCode2026Public)
  - Contains `ascope_assets/` folder with AdvantageScope configuration

### Option 2: Convert Your Team's CAD

1. **Export from Onshape** (or other CAD software)
   - Right-click assembly → Export → STEP format

2. **Convert STEP to GLB**
   - Download [CAD Assistant](https://www.opencascade.com/products/cad-assistant/) (free)
   - Open STEP file
   - Enable "Merge faces" option
   - Export as GLB

3. **Configure in AdvantageScope**
   - Place `robot.glb` in a custom assets folder
   - In AdvantageScope: **Help > Show Assets Folder**
   - Create folder structure: `robots/YourRobot/model.glb`
   - Select your robot in the 3D Field settings

### CAD Conversion Resources

- [AdvantageScope Custom Assets Guide](https://docs.advantagescope.org/more-features/custom-assets/)
- [GLTF Conversion Guide](https://docs.advantagescope.org/more-features/custom-assets/gltf-convert/)

## Physics Parameters

The simulation uses these FUEL game piece parameters (from `Constants.SimulationConstants`):

| Parameter | Value | Description |
|-----------|-------|-------------|
| Diameter | 15.0 cm (5.91 in) | FUEL ball diameter |
| Mass | 0.2 kg (0.45 lbs) | FUEL ball mass |
| Hood Angle | 55° | Fixed shooter hood angle |
| Launch Height | 0.45 m | Height of shooter exit |
| Flywheel Radius | 0.05 m (2 in) | For exit velocity calculation |
| Exit Velocity Multiplier | 0.85 | Accounts for slip |

### Exit Velocity Formula

```
exitVelocity = 2π × RPS × flywheelRadius × multiplier
```

At 50 RPS with default parameters:
```
exitVelocity = 2π × 50 × 0.05 × 0.85 ≈ 13.4 m/s
```

## Troubleshooting

### No Trajectories Visible in AdvantageScope
- Verify NetworkTables connection (check status bar)
- Ensure `Shooter/SuccessfulShot` and `Shooter/MissedShot` are added to 3D view
- Check that `SimulatedArena.getInstance().simulationPeriodic()` is called in `Robot.simulationPeriodic()`

### Vision Not Detecting AprilTags
- Ensure robot is positioned where tags are visible
- Check that `VisionSimSubsystem.simulationPeriodic(pose)` is called with current robot pose
- Verify AprilTag field layout loaded (check console for warnings)

### Shots Always Miss
- Tune the distance-to-RPS table values
- Check hood angle in `SimulationConstants`
- Verify target position (`HIGH_HUB_CENTER` in FuelProjectile)

## File Reference

| File | Purpose |
|------|---------|
| `VisionSimSubsystem.java` | PhotonVision AprilTag simulation |
| `FuelProjectile.java` | Projectile physics and trajectory logging |
| `Constants.SimulationConstants` | Physics parameters |
| `ShooterSubsystem.java` | Distance-to-RPS lookup table |
| `Robot.java` | Simulation initialization and periodic |

## Additional Resources

- [PhotonVision Simulation Docs](https://docs.photonvision.org/en/latest/docs/simulation/simulation.html)
- [Maple-sim Projectiles Guide](https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-projectiles/)
- [AdvantageScope Documentation](https://docs.advantagescope.org/)
- [Team 6328 Open Alliance Build Thread](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595)
