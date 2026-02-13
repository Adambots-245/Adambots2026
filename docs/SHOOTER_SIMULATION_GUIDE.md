# 3D Shooter Simulation Guide

This guide explains how to use the FUEL shooter simulation with AdvantageScope for 3D visualization and testing.

## Overview

The simulation system provides:
- **PhotonVision simulation** - Simulated AprilTag detection for distance-based shooting
- **Projectile physics** - Realistic ball trajectories using maple-sim
- **3D visualization** - AdvantageScope integration for trajectory display
- **Distance-to-RPS tuning** - Interpolating lookup table for shot calibration
- **Dashboard test commands** - Shuffleboard buttons for triggering shots in simulation

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
2. On the left sidebar, click the **"+"** button to add poses:
   - **Robot**: source = `SmartDashboard/Field/Robot` (or your swerve pose), type = **Robot**
3. Fire a test shot first (click **Sim/Shoot Default** in Shuffleboard) so the trajectory topics appear in NetworkTables
4. Click **"+"** again to add trajectory poses:
   - Source = `Shooter/SuccessfulShot`, type = **Trajectory**, color = green
   - Source = `Shooter/MissedShot`, type = **Trajectory**, color = red

   > **Note:** These NT entries only appear after the first shot is fired, since that's when `FuelProjectile` first publishes to them.

### 4. Trigger Test Shots

Use the Shuffleboard/SmartDashboard buttons (see [Test Commands](#simulation-test-commands) below):
- **Sim/Shoot Default** - Shoot at 59.8 RPS (3586 RPM)
- **Sim/Shoot Vision** - Shoot using vision distance lookup
- **Sim/Shoot Custom RPS** - Shoot at a custom RPS value
- **Sim/Shoot At Distance** - Shoot using RPS from distance lookup table

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

## Scoring Geometry (2026 REBUILT)

| Parameter | Value | Source |
|-----------|-------|--------|
| Hub opening height | 72 in (1.83 m) | Game manual |
| Hub opening shape | 41.7 in hexagonal | Game manual |
| Hub from alliance wall | 158.6 in (4.03 m) | Game manual |
| Barge (hanging) | Center of field | Game manual |
| Max scoring distance | ~167 in (~4.24 m) | Calculated (barge to hub) |
| Scoring range | 1.0 m - 4.5 m | Practical range |

## Simulation Test Commands

The following commands are available in Shuffleboard/SmartDashboard for testing (defined in `RobotContainer.configureDashboard()`):

| Command | Description | Parameters |
|---------|-------------|------------|
| `Sim/Shoot Default` | Shoots at 59.8 RPS (3586 RPM) | None |
| `Sim/Shoot Vision` | Uses vision distance to auto-calculate RPS | Requires visible AprilTag |
| `Sim/Shoot Custom RPS` | Shoots at user-specified RPS | Set `Sim/CustomRPS` first |
| `Sim/Shoot At Distance` | Looks up RPS from distance table | Set `Sim/TestDistance` first |
| `Sim/Estimate RPS` | Calculates ideal RPS for distance (no shot) | Set `Sim/TestDistance` first |

### Telemetry Outputs

After each shot, the following values are published:
- `Shooter/ExitVelocity` - Ball exit velocity (m/s)
- `Shooter/LaunchAngleDeg` - Hood angle (60°)
- `Shooter/RobotX`, `Shooter/RobotY` - Robot position at shot time
- `Sim/LastShotDistance` - Distance used (vision shots)
- `Sim/LastShotRPS` - RPS used
- `Sim/LastExitVelocity` - Calculated exit velocity
- `Sim/EstimatedRPS` - Physics-based RPS estimate

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
FuelProjectile.launch(robotPose, swerve.getRobotVelocity(), exitVelocity);
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
    // Range: 1.0m (close) to 4.5m (max scoring distance from barge)
    distanceToRPS.put(1.0, 33.0);   // Close shot
    distanceToRPS.put(1.5, 27.0);
    distanceToRPS.put(2.0, 27.0);
    distanceToRPS.put(2.5, 29.0);   // Mid shot
    distanceToRPS.put(3.0, 30.0);
    distanceToRPS.put(3.5, 32.0);
    distanceToRPS.put(4.0, 33.0);   // Far shot
    distanceToRPS.put(4.5, 35.0);   // Maximum range (near barge)
}
```

**Tuning Process:**
1. Run `./gradlew simulateJava` and connect AdvantageScope
2. Position the robot at a known distance in simulation
3. Set `Sim/TestDistance` to the distance and click **Sim/Shoot At Distance**
4. Observe the trajectory in AdvantageScope 3D view
5. If shot misses, adjust RPS: use **Sim/Shoot Custom RPS** to test different values
6. Update the `distanceToRPS` table in `ShooterSubsystem.java` with calibrated values
7. Repeat at multiple distances across the 1.0-4.5m range

## Physics Parameters

The simulation uses these parameters (from `Constants.SimulationConstants`):

| Parameter | Value | Description | Status |
|-----------|-------|-------------|--------|
| Diameter | 15.0 cm (5.91 in) | FUEL ball diameter | Confirmed |
| Mass | 0.227 kg (0.5 lbs) | FUEL ball mass | Confirmed |
| Hood Angle | 60° | Fixed shooter hood angle | Confirmed |
| Launch Height | 0.45 m | Height of shooter exit | WAITING (exit height from mechanical) |
| Flywheel Radius | 0.0508 m (2 in) | For exit velocity calculation | Confirmed |
| Exit Velocity Multiplier | 0.85 | Accounts for slip | Estimate |
| Hub Height | 1.83 m (72 in) | High hub target height | From game manual |

### Hardware Configuration (Confirmed)

| Component | Specification |
|-----------|--------------|
| Motors | 2x Kraken X60 (FOC) |
| Gear Ratio | 1:1 (direct drive) |
| Current Limit | 40A fuse |
| Target RPM | 3586 RPM (59.8 RPS, 62% of max) |
| Shooter Wheel Radius | 2 inches (0.0508 m) |
| Shooter Wheel Weight | 2.2 lbs |
| Flywheel Radius | 2 inches (0.0508 m) |
| Flywheel Weight | 1.5 lbs |

### Exit Velocity Formula

```
exitVelocity = 2π * RPS * flywheelRadius * multiplier
```

At 59.8 RPS (default target) with confirmed parameters:
```
exitVelocity = 2π * 59.8 * 0.0508 * 0.85 ≈ 16.2 m/s (53.1 ft/s)
```

### Hub Target Position

The hub center is defined in `FuelProjectile.java`:
```java
Translation3d HIGH_HUB_CENTER = new Translation3d(8.23, 4.11, 1.83);
// X=8.23m, Y=4.11m (field center), Z=1.83m (72 inches)
```

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
- Tune the distance-to-RPS table values (use Sim/Shoot Custom RPS to experiment)
- Check hood angle in `SimulationConstants` (should be 60°)
- Verify hub target position (`HIGH_HUB_CENTER` in FuelProjectile.java)
- Try adjusting `kExitVelocityMultiplier` (default 0.85)
- Use **Sim/Estimate RPS** to get the ideal physics-based estimate, then add margin

### Shots Always Short
- Increase the exit velocity multiplier or the RPS
- The 0.85 multiplier accounts for slip - adjust based on simulation results

## File Reference

| File | Purpose |
|------|---------|
| `VisionSimSubsystem.java` | PhotonVision AprilTag simulation |
| `FuelProjectile.java` | Projectile physics and trajectory logging |
| `Constants.SimulationConstants` | Physics parameters (hood angle, mass, radius, hub height) |
| `Constants.ShooterConstants` | Flywheel PID, feed-forward, default velocity |
| `ShooterSubsystem.java` | Distance-to-RPS lookup table |
| `RobotContainer.java` | Simulation test commands (configureDashboard) |
| `Robot.java` | Simulation initialization and periodic |

## Additional Resources

- [PhotonVision Simulation Docs](https://docs.photonvision.org/en/latest/docs/simulation/simulation.html)
- [Maple-sim Projectiles Guide](https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-projectiles/)
- [AdvantageScope Documentation](https://docs.advantagescope.org/)
- [Team 6328 Open Alliance Build Thread](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595)
- [ReCalc Flywheel Calculator](https://www.reca.lc/flywheel) - See `docs/flywheel-calculator-guide.md`
