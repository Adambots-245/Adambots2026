# Shooter Subsystem Implementation Plan - REBUILT 2026

> **Note:** This plan tracks the implementation of the shooter subsystem including flywheel control, turret aiming, hopper, and vision-integrated tracking.

---

## Overview

The shooter team is responsible for the complete game piece launching system for the 2026 FRC game REBUILT. This includes:

### Shooter Components
- **Dual flywheel motors** for launching game pieces
- **Turret motor** for aiming left/right (need to determine max rotation angle)
- **Turret encoder** (REV Through Bore) for absolute position feedback
- **Vision integration** for automatic target tracking using a camera mounted on the Turret

### Hopper/Indexer Components
- **Carousel motor** for rotating and storing game pieces
- **Uptake motor** for feeding game pieces from carousel to shooter
- **Hopper sensor** (CANRange) for detecting game pieces in the hopper

### Hardware Configuration (Team 245)
| Component | Motor/Sensor | Notes |
|-----------|--------------|-------|
| **SHOOTER** | | |
| Left Flywheel | Kraken X60 (TalonFX) | Leader motor |
| Right Flywheel | Kraken X60 (TalonFX) | Follower, inverted |
| Turret | Kraken X44 (TalonFX) | Brake mode enabled |
| Turret Encoder | REV Through Bore | Absolute position feedback |
| **HOPPER** | | |
| Carousel | Kraken X60 (TalonFX) | Rotates game pieces |
| Uptake | Kraken X60 (TalonFX) | Feeds to shooter |
| Hopper Sensor | CANRange | Detects game pieces (CAN bus) |

### Control Strategy
| Mechanism | Control Type | Controller |
|-----------|-------------|------------|
| Flywheel | Velocity PID + Feed-Forward | WPILib PIDController + SimpleMotorFeedforward |
| Turret | Position PID | WPILib PIDController |
| Carousel | Open-loop (percent output) | Direct motor control |
| Uptake | Open-loop (percent output) | Direct motor control |

### State Machine (Tracking)
```
DISABLED → TRACKING ↔ SCANNING → FAILED
              ↑                      │
              └──────────────────────┘ (manual re-enable)
```

---
#### Assign week numbers or dates to these phases so that you can plan to get everything completed before the robot is ready
---

## Phase 1: Hardware Setup & Basic Control

### 1.1 Motor Configuration
**Owner: Shooter Subteam**
**File:** `src/main/java/com/adambots/RobotMap.java`

- [ ] Verify CAN IDs match physical wiring
- [ ] Configure left flywheel motor inversion if needed
- [ ] Configure right flywheel as follower of left (opposite direction):
  ```java
  // In RobotMap.java or ShooterSubsystem constructor:
  kShooterRightMotor.setStrictFollower(kShooterLeftMotorPort, true);  // true = inverted
  ```
- [ ] Configure turret motor brake mode 
- [ ] Test motors spin correct direction with basic percent output

### 1.2 Turret Encoder Setup
**Owner: Shooter + Electrical Subteams**
**Files:** `RobotMap.java`, `Constants.java`, `ShooterSubsystem.java`

- [ ] Wire REV Through Bore Encoder to RoboRIO DIO port
- [ ] Add encoder port to Constants.java:
  ```java
  public static final int kTurretEncoderPort = X;  // TODO: Assign DIO port
  ```
- [ ] Add encoder to RobotMap.java:
  ```java
  public static final ThroughBoreEncoder kTurretEncoder =
      new ThroughBoreEncoder(ShooterConstants.kTurretEncoderPort);
  ```
- [ ] Update ShooterSubsystem constructor to accept encoder
- [ ] Calibrate encoder offset (measure when turret faces forward):
  ```java
  public static final double kTurretEncoderOffset = 0.0;  // TODO: Measure
  ```
- [ ] Verify encoder reads correctly in SmartDashboard

### 1.3 Turret Soft Limits
**Owner: Shooter + Mechanical Subteams**
**File:** `ShooterSubsystem.java`

- [ ] Measure turret rotation limits from mechanical team (degrees)
- [ ] Add limits to Constants.java:
  ```java
  public static final double kTurretMinAngle = -90.0;  // TODO: Measure
  public static final double kTurretMaxAngle = 90.0;   // TODO: Measure
  ```
- [ ] Configure soft limits in ShooterSubsystem constructor:
  ```java
  turretMotor.configureSoftLimits(
      kTurretMinAngle / 360.0 * kTurretGearRatio,
      kTurretMaxAngle / 360.0 * kTurretGearRatio,
      true  // enabled
  );
  ```
- [ ] Test limits prevent over-rotation

---

## Phase 2: Flywheel Control Implementation

### 2.1 Implement Flywheel PID Loop
**Owner: Programming Subteam**
**File:** `ShooterSubsystem.java`

- [ ] Uncomment flywheel PID loop in `periodic()`:
  ```java
  if (flywheelSetpoint > 0) {
      double currentVelocity = leftMotor.getVelocity().in(RotationsPerSecond);
      double pidOutput = flywheelPID.calculate(currentVelocity, flywheelSetpoint);
      double ffOutput = flywheelFF.calculate(flywheelSetpoint);
      leftMotor.setVoltage(pidOutput + ffOutput);
  } else {
      leftMotor.set(0);
  }
  ```
- [ ] Implement `spinUpCommand()`:
  ```java
  return run(() -> {
      flywheelSetpoint = ShooterConstants.kDefaultVelocity;
  }).withName("SpinUp");
  ```
- [ ] Implement `stopFlywheelCommand()`:
  ```java
  return runOnce(() -> {
      flywheelSetpoint = 0;
  }).withName("StopFlywheel");
  ```
- [ ] Add telemetry in periodic():
  ```java
  SmartDashboard.putNumber("Shooter/FlywheelSetpoint", flywheelSetpoint);
  SmartDashboard.putNumber("Shooter/FlywheelVelocity",
      leftMotor.getVelocity().in(RotationsPerSecond));
  SmartDashboard.putBoolean("Shooter/AtSpeed", flywheelPID.atSetpoint());
  ```

### 2.2 Tune Flywheel PID
**Owner: Programming Subteam**
**File:** `Constants.java`

- [ ] Start with feed-forward only (kP=0, kV from calculation):
  ```java
  // kV = 12V / freeSpeedRPS
  // Kraken X60: 6000 RPM = 100 RPS → kV = 0.12
  public static final double kFlywheelFF = 0.12;
  ```
- [ ] Test spin-up time and steady-state accuracy
- [ ] Add small P gain if needed (start with 0.01)
- [ ] Tune tolerance for `atSetpoint()`:
  ```java
  public static final double kFlywheelTolerance = 2.0;  // RPS
  ```
- [ ] Document final tuned values

### 2.3 Implement Flywheel Triggers
**Owner: Programming Subteam**
**File:** `ShooterSubsystem.java`

- [ ] Implement `isAtSpeedTrigger()`:
  ```java
  return new Trigger(() ->
      flywheelSetpoint > 0 && flywheelPID.atSetpoint());
  ```
- [ ] Implement `isSpinningTrigger()`:
  ```java
  return new Trigger(() ->
      leftMotor.getVelocity().in(RotationsPerSecond) > 5.0);
  ```
- [ ] Implement `isIdleTrigger()`:
  ```java
  return new Trigger(() ->
      Math.abs(leftMotor.getVelocity().in(RotationsPerSecond)) < 1.0);
  ```

---

## Phase 3: Turret Control Implementation

### 3.1 Implement Turret Position Reading
**Owner: Shooter Subteam**
**File:** `ShooterSubsystem.java`

- [ ] Implement `getTurretAngle()` using Through Bore Encoder:
  ```java
  private double getTurretAngle() {
      double rawAngle = turretEncoder.getPosition().in(Degrees);  // 0-360
      double angle = rawAngle - ShooterConstants.kTurretEncoderOffset;
      // Normalize to -180 to +180
      if (angle > 180) angle -= 360;
      if (angle < -180) angle += 360;
      return angle;
  }
  ```
- [ ] Add turret angle to telemetry
- [ ] Verify angle reads correctly when turret rotates

### 3.2 Implement Turret PID Loop
**Owner: Programming Subteam**
**File:** `ShooterSubsystem.java`

- [ ] Uncomment turret PID loop in `periodic()`:
  ```java
  if (turretPIDEnabled) {
      double currentAngle = getTurretAngle();
      double pidOutput = turretPID.calculate(currentAngle, turretSetpointDegrees);
      turretMotor.set(MathUtil.clamp(pidOutput, -1, 1));
  }
  ```
- [ ] Add turret telemetry:
  ```java
  SmartDashboard.putNumber("Shooter/TurretAngle", getTurretAngle());
  SmartDashboard.putNumber("Shooter/TurretSetpoint", turretSetpointDegrees);
  SmartDashboard.putBoolean("Shooter/TurretOnTarget", turretPID.atSetpoint());
  SmartDashboard.putBoolean("Shooter/TurretPIDEnabled", turretPIDEnabled);
  ```

### 3.3 Tune Turret PID
**Owner: Programming Subteam**
**File:** `Constants.java`

- [ ] Start with low P gain (0.02-0.05)
- [ ] Test centering from various angles
- [ ] Add D gain if oscillating
- [ ] Tune tolerance for `atSetpoint()`:
  ```java
  public static final double kTurretAngleTolerance = 2.0;  // degrees
  ```
- [ ] Test with faster movements, adjust as needed
- [ ] Document final tuned values

### 3.4 Implement Turret Triggers
**Owner: Programming Subteam**
**File:** `ShooterSubsystem.java`

- [ ] Verify `isTurretAtTargetTrigger()` works (already implemented)
- [ ] Implement `isTurretCenteredTrigger()`:
  ```java
  return new Trigger(() -> Math.abs(getTurretAngle()) < 5.0);
  ```
- [ ] Implement limit triggers using soft limit values
- [ ] Implement `isReadyToFireTrigger()`:
  ```java
  return isAtSpeedTrigger().and(isTurretAtTargetTrigger());
  ```

---

## Phase 4: Hopper/Indexer Implementation

### 4.1 Hopper Motor Configuration
**Owner: Programming Subteam**
**File:** `RobotMap.java`

- [ ] Verify CAN IDs match physical wiring
- [ ] Test motors spin correct direction with basic percent output
- [ ] Configure current limits if needed:
  ```java
  kHopperCarouselMotor.setCurrentLimit(30);
  kHopperUptakeMotor.setCurrentLimit(30);
  ```

### 4.2 Hopper Sensor Setup
**Owner: Programming + Electrical Subteams**
**Files:** `RobotMap.java`, `HopperSubsystem.java`

- [ ] Wire CANRange sensor to CAN bus
- [ ] Assign CAN ID and update RobotMap.java:
  ```java
  // In RobotMap.java
  public static final CANRangeSensor kHopperSensor = new CANRangeSensor(kHopperSensorPort);
  ```
- [ ] Test sensor detects game pieces correctly
- [ ] Configure detection threshold distance if needed
- [ ] Verify sensor reads correctly in SmartDashboard

### 4.3 Implement Hopper Commands
**Owner: Programming Subteam**
**File:** `HopperSubsystem.java`

- [ ] Add hopper constants to Constants.java:
  ```java
  public static final class HopperConstants {
      public static final double kCarouselSpeed = 0.5;    // TODO: Tune
      public static final double kUptakeSpeed = 0.7;      // TODO: Tune
      public static final double kReverseSpeed = -0.5;    // For unjamming
      public static final double kIndexTimeout = 0.5;     // seconds
  }
  ```
- [ ] Implement `runCarouselCommand()`
- [ ] Implement `feedCommand()`
- [ ] Implement `stopCommand()`
- [ ] Implement `continuousFeedCommand()`
- [ ] Implement reverse commands for unjamming

### 4.4 Implement Hopper Triggers
**Owner: Shooter Subteam**
**File:** `HopperSubsystem.java`

- [ ] Implement `hasGamePieceTrigger()`
- [ ] Implement `isEmptyTrigger()` (already uses negate of hasGamePiece)
- [ ] Implement `isFullTrigger()` if needed (may need additional sensor)
- [ ] Add telemetry in periodic():
  ```java
  SmartDashboard.putBoolean("Hopper/HasGamePiece", hopperSensor.isDetecting());
  SmartDashboard.putNumber("Hopper/CarouselCurrent", carouselMotor.getCurrent());
  SmartDashboard.putNumber("Hopper/UptakeCurrent", uptakeMotor.getCurrent());
  ```

### 4.5 Tune Hopper Speeds
**Owner: Shooter Subteam**

- [ ] Test carousel speed - should move game pieces smoothly
- [ ] Test uptake speed - should feed reliably without jamming
- [ ] Test reverse speeds for unjamming
- [ ] Tune `indexOneCommand()` timeout for consistent indexing
- [ ] Document final tuned values

---

## Phase 5: Vision Integration (Coordinate with Vision Subteam)

### 4.1 Wire Vision to Turret Tracking
**Owner: Shooter + Vision Subteams**
**File:** `RobotContainer.java`

- [ ] Create vision suppliers in RobotContainer
- [ ] Bind tracking command to operator button

### 4.2 Implement TurretTrackingCommands
**Owner: Shooter Subteam**
**File:** `TurretTrackingCommands.java`

- [ ] Complete `trackTargetCommand()`
- [ ] Test tracking with printed AprilTags
- [ ] Test tracking while moving robot

### 4.3 Implement Auto-Track with Scanning
**Owner: Shooter Subteam**
**Files:** `ShooterSubsystem.java`, `TurretTrackingCommands.java`

- [ ] Implement state machine transitions in ShooterSubsystem
- [ ] Complete `autoTrackCommand()` in TurretTrackingCommands
- [ ] Add target lost counter for debouncing
- [ ] Test scan-to-reacquire behavior

---

## Phase 6: Shooting Integration

### 5.1 Test ShootCommands
**Owner: Shooter Subteam**
**File:** `ShootCommands.java`

- [ ] Verify `shootCommand()` sequence works:
  1. Flywheel spins up
  2. Waits for at-speed
  3. Hopper feeds
  4. Both stop
- [ ] Adjust feed timeout if needed
- [ ] Test `shootAllCommand()` for continuous shooting

### 5.2 Add Shooting Button Bindings
**Owner: Programming Subteam**
**File:** `RobotContainer.java`

- [ ] Bind shoot command
- [ ] Bind spin-up (pre-spin) command
- [ ] Bind manual turret control

### 5.3 LED Integration
**Owner: Shooter Subteam**
**File:** `RobotContainer.java`

- [ ] Show green when ready to fire
- [ ] Show yellow when spinning up
- [ ] Show orange when tracking (target visible)

---

## Phase 7: Prototype Testing (Before New Robot Ready)

> **Purpose:** Test flywheel control and basic turret functionality on the prototype before the full robot is ready.

### 6.1 Prototype Shooter Setup
**Owner: Mechanical + Shooter Subteams**

- [ ] Mount test flywheel assembly (if available)
- [ ] Wire motors to a test board or ChassisBot RoboRIO
- [ ] Update CAN IDs if different from planned values
- [ ] Deploy and test basic motor control

### 6.2 Prototype Flywheel Validation
**Owner: Shooter Subteam**

- [ ] Test flywheel spin-up time
- [ ] Verify PID + FF achieves target velocity
- [ ] Measure velocity stability (should be within tolerance)
- [ ] Test `isAtSpeedTrigger()` behavior

### 6.3 Prototype Turret Validation (if mounted)
**Owner: Shooter + Mechanical Subteams**

- [ ] Test encoder reads correctly
- [ ] Test PID position control
- [ ] Verify soft limits work
- [ ] Test aiming commands

---

## Phase 8: Full Robot Integration

### 7.1 Final Hardware Installation
**Owner: Mechanical + Electrical + Shooter Subteams**

- [ ] Install shooter assembly on robot
- [ ] Route wiring with strain relief
- [ ] Verify CAN IDs match RobotMap
- [ ] Calibrate turret encoder offset

### 7.2 Final Tuning
**Owner: Shooter Subteam**

- [ ] Re-tune flywheel PID with actual game pieces
- [ ] Re-tune turret PID with full robot weight
- [ ] Tune tracking PID (if different from bench test)
- [ ] Set final velocity values based on shooting distance

### 7.3 Autonomous Integration
**Owner: Shooter + Drivetrain Subteams**
**File:** `RobotContainer.java`

- [ ] Register PathPlanner named commands:
  ```java
  NamedCommands.registerCommand("spinUp", shooter.spinUpCommand());
  NamedCommands.registerCommand("shoot", ShootCommands.shootCommand(shooter, hopper));
  NamedCommands.registerCommand("aimCenter", shooter.centerTurretCommand());
  ```
- [ ] Test shooting during autonomous paths

---
