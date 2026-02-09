# Intake Subsystem Implementation Plan - REBUILT 2026

> **Note:** This plan tracks the implementation of the intake subsystem including game piece acquisition, deploy/retract mechanism, and handoff to hopper.

---

## Overview

The intake team is responsible for the game piece acquisition system for the 2026 FRC game REBUILT. This includes:

### Intake Components
- **Intake rollers/wheels** for grabbing game pieces
- **Deploy/pivot motor** for extending intake beyond frame perimeter
- **Pivot encoder** for position feedback (if using closed-loop control)
- **Intake sensor** for detecting game piece in intake

### Hardware Configuration (Team 245)
| Component | Motor/Sensor | Notes |
|-----------|--------------|-------|
| Intake Rollers | TBD | Game piece acquisition |
| Deploy/Pivot | TBD | Extends intake outside frame |
| Pivot Encoder | TBD (if needed) | Position feedback for deploy |
| Intake Sensor | TBD | Detects game piece presence |

### Control Strategy
| Mechanism | Control Type | Controller |
|-----------|-------------|------------|
| Intake Rollers | Open-loop (percent output) | Direct motor control |
| Deploy/Pivot | Position PID or Open-loop | WPILib PIDController (if closed-loop) |

### State Machine (Intake)
```
STOWED → DEPLOYING → DEPLOYED → INTAKING → HANDOFF → STOWED
                         ↓
                     RETRACTING
```

---
#### Assign week numbers or dates to these phases so that you can plan to get everything completed before the robot is ready
---

## Phase 1: Hardware Setup & Basic Control

### 1.1 Motor Configuration
**Owner: Intake Subteam**
**File:** `RobotMap.java`

- [ ] Verify CAN IDs match physical wiring
- [ ] Configure intake roller motor inversion if needed
- [ ] Configure deploy/pivot motor inversion if needed
- [ ] Configure brake mode on deploy motor (hold position when stopped)
- [ ] Test motors spin correct direction with basic percent output

### 1.2 Pivot Encoder Setup (If Using Closed-Loop)
**Owner: Intake + Electrical Subteams**
**Files:** `RobotMap.java`, `Constants.java`, `IntakeSubsystem.java`

- [ ] Wire encoder to motor controller or RoboRIO
- [ ] Add encoder configuration to RobotMap
- [ ] Calibrate encoder offset (measure when intake is stowed)
- [ ] Verify encoder reads correctly in SmartDashboard

### 1.3 Deploy Soft Limits
**Owner: Intake + Mechanical Subteams**
**File:** `IntakeSubsystem.java`

- [ ] Measure deploy rotation limits from mechanical team
- [ ] Add limits to Constants.java
- [ ] Configure soft limits in IntakeSubsystem
- [ ] Test limits prevent over-rotation

---

## Phase 2: Deploy/Pivot Control

### 2.1 Implement Deploy Commands
**Owner: Intake Subteam**
**File:** `IntakeSubsystem.java`

- [ ] Implement `deployCommand()` - extends intake
- [ ] Implement `retractCommand()` - stows intake
- [ ] Implement `toggleDeployCommand()` - switches between states
- [ ] Add telemetry for deploy position/state

### 2.2 Tune Deploy Control (If Using PID)
**Owner: Intake Subteam**
**File:** `Constants.java`

- [ ] Start with low P gain
- [ ] Test deploying from stowed position
- [ ] Test retracting from deployed position
- [ ] Add D gain if oscillating
- [ ] Tune tolerance for position setpoints
- [ ] Document final tuned values

### 2.3 Implement Deploy Triggers
**Owner: Intake Subteam**
**File:** `IntakeSubsystem.java`

- [ ] Implement `isDeployedTrigger()`
- [ ] Implement `isStowedTrigger()`
- [ ] Implement `isMovingTrigger()`

---

## Phase 3: Intake Roller Control

### 3.1 Implement Roller Commands
**Owner: Intake Subteam**
**File:** `IntakeSubsystem.java`

- [ ] Add intake constants to Constants.java:
  - Intake speed
  - Outtake/eject speed
  - Slow intake speed (for handoff)
- [ ] Implement `intakeCommand()` - runs rollers inward
- [ ] Implement `outtakeCommand()` - runs rollers outward (eject)
- [ ] Implement `stopRollersCommand()`
- [ ] Add telemetry for roller state/current

### 3.2 Tune Roller Speeds
**Owner: Intake Subteam**

- [ ] Test intake speed with actual game pieces
- [ ] Adjust speed for reliable acquisition
- [ ] Test outtake speed for ejecting
- [ ] Test handoff speed to hopper
- [ ] Document final tuned values

---

## Phase 4: Sensor Integration

### 4.1 Intake Sensor Setup
**Owner: Intake + Electrical Subteams**
**Files:** `RobotMap.java`, `IntakeSubsystem.java`

- [ ] Wire sensor (beam break, proximity, or CANRange)
- [ ] Add sensor to RobotMap
- [ ] Test sensor detects game pieces correctly
- [ ] Configure detection threshold if needed
- [ ] Verify sensor reads correctly in SmartDashboard

### 4.2 Implement Sensor Triggers
**Owner: Intake Subteam**
**File:** `IntakeSubsystem.java`

- [ ] Implement `hasGamePieceTrigger()`
- [ ] Implement `isEmptyTrigger()`
- [ ] Add telemetry for sensor state

### 4.3 Automatic Intake Stop
**Owner: Intake Subteam**
**File:** `IntakeSubsystem.java`

- [ ] Implement auto-stop when game piece detected
- [ ] Add configurable delay before stopping (debounce)
- [ ] Test reliability with repeated intakes

---

## Phase 5: Intake Commands & Sequences

### 5.1 Create Composite Commands
**Owner: Intake Subteam**
**File:** `IntakeCommands.java` (or `IntakeSubsystem.java`)

- [ ] Implement `autoIntakeCommand()`:
  - Deploy intake
  - Run rollers
  - Stop when game piece detected
  - Retract intake
- [ ] Implement `ejectCommand()`:
  - Deploy intake
  - Run rollers reverse
  - Stop after timeout or button release
- [ ] Implement `groundPickupCommand()` (deploy + intake until acquired)

### 5.2 Add Button Bindings
**Owner: Intake Subteam**
**File:** `RobotContainer.java`

- [ ] Bind intake command to operator button
- [ ] Bind outtake/eject command
- [ ] Bind manual deploy/retract controls
- [ ] Bind emergency stop

---

## Phase 6: Hopper Handoff Integration

### 6.1 Coordinate with Shooter/Hopper Team
**Owner: Intake + Shooter Subteams**

- [ ] Define handoff sequence with hopper team
- [ ] Determine timing/speeds for reliable handoff
- [ ] Agree on sensor-based vs time-based handoff

### 6.2 Implement Handoff Command
**Owner: Intake Subteam**
**File:** `IntakeCommands.java`

- [ ] Implement `handoffToHopperCommand()`:
  - Run intake rollers at handoff speed
  - Wait for hopper sensor or timeout
  - Stop intake rollers
- [ ] Test handoff reliability
- [ ] Tune timing as needed

### 6.3 Full Acquisition Sequence
**Owner: Intake + Shooter Subteams**
**File:** `RobotContainer.java` or `IntakeCommands.java`

- [ ] Create full sequence: deploy → intake → handoff → stow
- [ ] Test complete cycle timing
- [ ] Integrate with hopper carousel if needed

---

## Phase 7: Prototype Testing (Before New Robot Ready)

> **Purpose:** Test intake control and game piece handling on a prototype before the full robot is ready.

### 7.1 Prototype Intake Setup
**Owner: Mechanical + Intake Subteams**

- [ ] Mount test intake assembly (if available)
- [ ] Wire motors to test board or chassisbot
- [ ] Update CAN IDs if different from planned values
- [ ] Deploy and test basic motor control

### 7.2 Prototype Validation
**Owner: Intake Subteam**

- [ ] Test deploy/retract motion
- [ ] Test roller intake with game pieces
- [ ] Verify sensor detection works
- [ ] Test auto-stop on game piece detection
- [ ] Document any issues or tuning needed

---

## Phase 8: Full Robot Integration

### 8.1 Final Hardware Installation
**Owner: Mechanical + Electrical + Intake Subteams**

- [ ] Install intake assembly on robot
- [ ] Route wiring with strain relief
- [ ] Verify CAN IDs match RobotMap
- [ ] Calibrate encoder offset (if applicable)

### 8.2 Final Tuning
**Owner: Intake Subteam**

- [ ] Re-tune deploy PID with full robot weight
- [ ] Re-tune intake speeds with competition game pieces
- [ ] Verify handoff works with actual hopper
- [ ] Set final speed values

### 8.3 Autonomous Integration
**Owner: Intake + Drivetrain Subteams**
**File:** `RobotContainer.java`

- [ ] Register PathPlanner named commands:
  - `deployIntake`
  - `autoIntake`
  - `stowIntake`
- [ ] Test intake during autonomous paths
- [ ] Tune timing for auto game piece pickup

### 8.4 LED Integration
**Owner: Intake Subteam**
**File:** `RobotContainer.java`

- [ ] Show color when intake deployed
- [ ] Show color when game piece acquired
- [ ] Flash when handoff in progress

---

## Key Files Reference

| File | Purpose |
|------|---------|
| `IntakeSubsystem.java` | Subsystem with motors, sensors, commands |
| `IntakeCommands.java` | Composite commands (if separate file) |
| `RobotMap.java` | Motor and sensor definitions |
| `Constants.java` | Intake speeds, PID values, limits |
| `RobotContainer.java` | Button bindings, named commands |

---
