# ReCalc Flywheel Calculator Guide

> **Calculator Link:** https://www.reca.lc/flywheel
>
> Use this calculator to determine initial flywheel control constants and validate
> shooter performance before tuning on the real robot.

---

## What This Calculator Does

The ReCalc Flywheel Calculator models the physics of a flywheel shooter to predict:
- The maximum RPM the flywheel can achieve given your motor, ratio, and current limit
- How long it takes to reach target speed (windup time)
- How long it takes to recover after shooting (recovery time)
- How fast the game piece exits the shooter (projectile speed)
- Feed-forward and feedback control gains (kV, kA, kP)

These predictions help us set initial values in `Constants.java` so we don't start
tuning from scratch on the robot.

---

## Our Hardware (Confirmed)

| Component | Value |
|-----------|-------|
| Flywheel Motors | **2x Kraken X60 (FOC)** - one on each side |
| Free Speed | 6000 RPM (100 RPS) |
| Stall Torque | 7.09 Nm |
| Current Limit | 40A per motor |
| Gear Ratio | 1:1 (direct drive, no step-up or reduction) |
| Shooter Wheel Radius | 2 inches |
| Shooter Wheel Weight | 2.2 lbs |
| Flywheel Radius | 2 inches (same shaft as shooter) |
| Flywheel Weight | 1.5 lbs |
| Hood Angle | 60 degrees (fixed) |
| Target RPM | 3586 (62% of max under 40A limit) |

---

## Step-by-Step Instructions

### Step 1: Motor Configuration

| Field | What to Enter | Our Value |
|-------|--------------|-----------|
| Motor Count | Number of flywheel motors | **2** |
| Motor Type | Select from dropdown | **Kraken X60 (FOC)** |
| Efficiency (%) | Start at 100%, reduce if predictions are optimistic | **100** (adjust later) |
| Current Limit (A) | Must match your code/firmware limit | **40** |

### Step 2: Gear Ratio

| Field | What to Enter | Our Value |
|-------|--------------|-----------|
| Shooter Ratio | Gear ratio between motor and flywheel wheel | **1** |
| Ratio Type | Neither - direct drive | **Reduction** (1:1 is the same either way) |

With a 1:1 ratio, the flywheel spins at the same speed as the motor output shaft.

### Step 3: Understand Shooter Max Speed

The calculator shows a **Shooter Max Speed** field (greyed out / computed). This is
the theoretical maximum RPM the flywheel can achieve given your motor, gear ratio,
and current limit. It is NOT something we set - it's what the calculator tells us
our hardware is capable of.

With our configuration (2x Kraken X60 FOC, 1:1, 40A limit), the max speed is
approximately **5784 RPM**. Our target of 3586 RPM is 62% of this, leaving
plenty of headroom for the motor to maintain speed under load.

### Step 4: Target Speed

| Field | What to Enter | Our Value |
|-------|--------------|-----------|
| Target Shooter RPM | Desired flywheel speed for a specific shot | **3586** (nominal) |

Our shooter uses **vision-based distance** from AprilTags on the hub to determine
the target RPM for each shot. There is no single fixed target RPM - the speed changes
based on how far we are from the hub. The 3586 RPM value is the nominal/default.

**How to use the calculator for this:** Run the calculator at multiple target RPMs
to understand windup/recovery behavior across your operating range. See the
"Distance-to-RPS Mapping" section below for details.

For our scoring range (~1.5m to ~4.5m), try these RPM values:
- Close shot (~1.5m): ~2400 RPM (40 RPS)
- Mid shot (~2.5m): ~3000 RPM (50 RPS)
- Default shot (~3.0m): ~3586 RPM (59.8 RPS)
- Far shot (~4.0m): ~4200 RPM (70 RPS)
- Max range (~4.5m): ~4500 RPM (75 RPS)

### Step 5: Projectile (FUEL) Configuration

| Field | What to Enter | 2026 REBUILT FUEL Value |
|-------|--------------|------------------------|
| Projectile Weight | Weight of the game piece | **0.5 lbs** (worst case) |

**2026 REBUILT FUEL Specifications:**
- Diameter: **5.91 inches**
- Weight: **0.448 - 0.5 lbs** (using 0.5 lbs for worst case)
- Material: Foam ball
- Note: Compression varies between batches but is within spec

Source: [AndyMark Official REBUILT FUEL](https://andymark.com/products/official-rebuilt-fuel),
[2026 Game Manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf)

### Step 6: Shooter Wheel Properties

| Field | What to Enter | Our Value |
|-------|--------------|-----------|
| Shooter Radius | Radius of the flywheel wheel that contacts the FUEL | **2 inches** |
| Shooter Weight | Weight of the flywheel wheel | **2.2 lbs** |

> If the mechanical team provides a MOI (Moment of Inertia) value from CAD, check
> "Use Custom Shooter MOI" and enter it directly. This is more accurate than
> letting the calculator estimate from radius + weight.

### Step 7: Flywheel Properties

The flywheel is on the **same shaft** as the shooter wheel (1:1 ratio).

| Field | What to Enter | Our Value |
|-------|--------------|-----------|
| Flywheel Radius | Radius of the flywheel mass | **2 inches** |
| Flywheel Weight | Weight of the flywheel mass | **1.5 lbs** |
| Flywheel:Shooter Ratio | Same shaft | **1 Reduction** (1:1) |

### Step 8: Read the Outputs

After entering all inputs, the calculator produces:

| Output | What It Means | How We Use It |
|--------|---------------|---------------|
| **Shooter Max Speed** | Maximum RPM the hardware can achieve (~5784 RPM) | Sanity check: all target RPMs must be below this |
| **Estimated Windup Time** | Time from 0 to target RPM | Spin-up command timeout, auto timing |
| **Estimated Recovery Time** | Time to recover RPM after a shot | Minimum time between shots in auto |
| **Shooter Wheel Surface Speed** | Linear speed at wheel edge (ft/s) | Sanity check against desired projectile speed |
| **Estimated Projectile Speed** | FUEL exit velocity (ft/s) | Validate scoring distance is achievable |
| **Speed After Shot** | RPM immediately after shooting | How much speed drops per shot |
| **Flywheel Energy (J)** | Kinetic energy stored | Safety consideration |
| **Projectile Energy (J)** | Energy transferred to FUEL | Efficiency of energy transfer |

### Step 9: Record the Control Gains

The most valuable output for our code:

| Gain | What It Is | Maps to in Constants.java |
|------|-----------|--------------------------|
| **kV** | Feed-forward velocity gain | `kFlywheelFF` (after unit conversion) |
| **kA** | Feed-forward acceleration gain | Could add to ShooterConstants (optional) |
| **kP** | Proportional feedback gain | `kFlywheelKP` |

**For kP:** You must enter Loop Time (20 ms for standard FRC) and Measurement Delay
(depends on sensor, typically 10-25 ms for TalonFX velocity) for the calculator to
compute kP. The calculator warns that these gains depend critically on these values.

---

## Distance-to-RPS Mapping

Our shooter determines how fast to spin the flywheel based on the distance to the
hub, measured by PhotonVision using AprilTags. The vision system provides distance
in meters, and we need to convert that to a target flywheel RPS.

### Scoring Geometry (Confirmed)

| Parameter | Value | Source |
|-----------|-------|--------|
| Hub opening height | **72 inches (1.83m)** | Game manual |
| Hub opening size | **41.7 inch hexagonal** | Game manual |
| Hub distance from alliance wall | **158.6 inches (4.03m)** | Game manual |
| Hood/exit angle | **60 degrees** (fixed) | Mechanical team |
| Exit height | **TBD** (waiting from mechanical) | Mechanical team |
| Max scoring distance | **~4.24m (~167 inches)** | Barge area to hub |
| Min scoring distance | **~1.0m** | Close to hub |

### How It Works in Our Code

`ShooterSubsystem.java` has an `InterpolatingDoubleTreeMap` that maps distance (m)
to flywheel RPS. WPILib's interpolation automatically provides smooth values between
calibrated points:

```java
private static final InterpolatingDoubleTreeMap distanceToRPS = new InterpolatingDoubleTreeMap();
static {
    distanceToRPS.put(1.0, XX.X);   // Close shot
    distanceToRPS.put(1.5, XX.X);
    distanceToRPS.put(2.0, XX.X);
    distanceToRPS.put(2.5, XX.X);   // Mid shot
    distanceToRPS.put(3.0, XX.X);
    distanceToRPS.put(3.5, XX.X);
    distanceToRPS.put(4.0, XX.X);   // Far shot
    distanceToRPS.put(4.5, XX.X);   // Maximum range
}
```

### Using the Calculator to Build This Table

Run the calculator multiple times at different Target RPMs and record the
**Estimated Projectile Speed** for each. Then work backwards to find the RPM
needed for each distance:

1. **Determine required projectile speed for each distance.**
   Use basic projectile physics: to reach the hub at distance `d` meters with
   exit angle `theta` and exit height `h`:

   ```
   v = sqrt( (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - (hub_height - h))) )
   ```

   With our confirmed values:
   - `g` = 9.81 m/s²
   - `theta` = 60 degrees
   - `hub_height` = 1.83m (72 inches)
   - `h` = **TBD** (exit height from floor in meters)

2. **For each distance, find the RPM that produces that projectile speed.**
   Enter different Target RPM values in the calculator until the
   **Estimated Projectile Speed** matches the required speed from step 1.

3. **Record in the table:**

   | Distance (m) | Required Projectile Speed | Target RPM | Target RPS |
   |--------------|--------------------------|------------|------------|
   | 1.0 | | | |
   | 1.5 | | | |
   | 2.0 | | | |
   | 2.5 | | | |
   | 3.0 | | | |
   | 3.5 | | | |
   | 4.0 | | | |
   | 4.5 | | | |

> **Note:** Once the exit height is confirmed, we can pre-calculate the required
> projectile speeds and fill in this table.

### Formula vs Lookup Table

**Preferred approach: Formula.**
If the distance-to-RPS relationship follows a predictable curve (e.g., square root,
polynomial), we can derive a formula from the data points. This is cleaner and
handles any distance without needing exact table entries.

For example, if the data fits a square-root relationship:
```java
public double getRPSForDistance(double distanceMeters) {
    return a * Math.sqrt(distanceMeters) + b;
}
```

Or a polynomial:
```java
public double getRPSForDistance(double distanceMeters) {
    return a * distanceMeters * distanceMeters + b * distanceMeters + c;
}
```

**How to derive the formula:**
1. Collect the distance-to-RPS data points (from calculator + on-robot tuning)
2. Plot them in a spreadsheet or tool like Desmos
3. Fit a curve (polynomial regression, square root, etc.)
4. If R² > 0.99, the formula is good enough - use it
5. If R² is low or the relationship is irregular, stick with the lookup table

**Fallback: Interpolating lookup table.**
The current `InterpolatingDoubleTreeMap` approach works well and provides smooth
interpolation between calibrated points. The downside is you need enough data points
to cover the full range, and behavior outside the table range relies on extrapolation.

**Recommendation:** Start with the lookup table during prototyping (easy to add/modify
entries). Once you have enough data, try fitting a formula. If it works, switch to
the formula for cleaner code. If not, the lookup table is perfectly fine.

### Validation Process

1. Place robot at known distances (1.5m, 2.5m, 3.5m, 4.5m) using tape measure
2. Use vision to confirm distance reading matches
3. Shoot at each distance and record if FUEL hits the hub
4. Adjust RPS up/down for each distance until consistent scoring
5. Update the lookup table or formula with tuned values
6. Run the calculator at each tuned RPM to verify the predicted projectile
   speed makes sense for that distance

---

## Unit Conversion for Our Codebase

The calculator outputs kV in **V*s/m** (volts per meter-per-second of surface speed).
Our code uses **V/RPS** (volts per rotation-per-second of the flywheel).

### Conversion Formula

```
kV_rotational = kV_linear * (2 * pi * radius_meters)
```

**With our wheel radius (2 inches = 0.0508m):**
- Calculator kV = X.XX V*s/m (read from calculator output)
- kV_rotational = X.XX * (2 * 3.14159 * 0.0508) = **X.XXX V/RPS**

### Current vs Calculator Comparison

Our current approach in `Constants.java`:
```
kFlywheelFF = 12.0 / kFlywheelMaxFreeSpeedRPS = 12.0 / 100.0 = 0.12 V/RPS
```

This is a simplified estimate (assumes no load, no gear ratio, no friction).
The calculator accounts for gear ratio, flywheel inertia, current limits, and
projectile mass, so its value will be more accurate.

---

## Information Needed from Mechanical Team

Use this checklist when meeting with the mechanical team to gather the parameters
needed for the calculator:

### Flywheel Mechanism
- [x] **Flywheel wheel diameter** - 4 inches (2 inch radius)
- [x] **Flywheel wheel weight** - 2.2 lbs
- [ ] **Flywheel wheel material** - affects grip and compression on the FUEL
- [ ] **Flywheel MOI** (if available from CAD) - more accurate than radius+weight estimate
- [x] **Number of flywheel wheels** - 2 (left + right, driven by 2x Kraken X60)

### Gear Ratio
- [x] **Gear ratio** - 1:1 direct drive
- [x] **Ratio type** - no step-up or reduction
- [ ] **Belt/gear/chain drive** - type of power transmission (affects efficiency %)

### Additional Flywheel Mass
- [x] **Flywheel on same shaft as shooter** - yes, 1:1
- [x] **Flywheel mass diameter** - 4 inches (2 inch radius)
- [x] **Flywheel mass weight** - 1.5 lbs

### Shooter Geometry (Needed for Distance-to-RPS Calculations)
- [ ] **Exit height** - height of FUEL center at exit point from floor (**WAITING**)
- [x] **Exit angle** - 60 degrees from horizontal (fixed hood)
- [ ] **FUEL contact point** - where on the wheel does the FUEL touch?
- [ ] **Compression** - how much is the FUEL compressed against the wheel?
- [x] **Hood angle adjustable?** - No, fixed at 60 degrees

### Scoring Target Geometry (From Game Manual)
- [x] **Hub opening height** - 72 inches (1.83m) from floor
- [x] **Hub opening size** - 41.7 inch hexagonal opening
- [x] **Max scoring distance** - ~4.24m (from barge/hanging area to hub)

---

## Changes Required in Our Codebase

After using the calculator, update the following:

### Constants.java - ShooterConstants

Replace the simplified feed-forward calculation with calculator-derived values:

```java
// Replace: public static final double kFlywheelFF = 12.0 / kFlywheelMaxFreeSpeedRPS;
// With calculator-derived value (after unit conversion):
public static final double kFlywheelKV = <converted_kV>;  // V/RPS from ReCalc
public static final double kFlywheelKA = <kA_value>;       // V/RPS² from ReCalc (optional)
```

Update the default velocity to match the confirmed target:

```java
public static final double kDefaultVelocity = 59.8;  // RPS (3586 RPM / 60)
```

Add timing constants for command sequencing:

```java
public static final double kFlywheelWindupTime = <value>;    // seconds, from calculator
public static final double kFlywheelRecoveryTime = <value>;  // seconds, from calculator
```

Update PID gains if the calculator provides kP (requires loop time + measurement delay):

```java
public static final double kFlywheelKP = <value>;  // from calculator with 20ms loop time
```

### ShooterSubsystem.java

Update the `distanceToRPS` lookup table with calculator-derived + tuned values
(range trimmed to 1.0m - 4.5m to match our actual scoring distances):

```java
static {
    // Values derived from ReCalc calculator + on-robot validation
    // Range: 1.0m (close to hub) to 4.5m (near barge/hanging area)
    distanceToRPS.put(1.0, <tuned_rps>);
    distanceToRPS.put(1.5, <tuned_rps>);
    distanceToRPS.put(2.0, <tuned_rps>);
    distanceToRPS.put(2.5, <tuned_rps>);
    distanceToRPS.put(3.0, <tuned_rps>);
    distanceToRPS.put(3.5, <tuned_rps>);
    distanceToRPS.put(4.0, <tuned_rps>);
    distanceToRPS.put(4.5, <tuned_rps>);
}
```

Or, if a formula fits the data well, replace `getRPSForDistance()`:

```java
public double getRPSForDistance(double distanceMeters) {
    // Formula derived from curve-fitting calibrated data points
    return a * Math.sqrt(distanceMeters) + b;
}
```

Update the `SimpleMotorFeedforward` with calculator-derived kV (and optionally kA):

```java
private final SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(kS, kV, kA);
```

> **Note:** `kS` (static friction) cannot be determined from the calculator. Keep it at 0
> or measure it on the real robot by slowly increasing voltage until the motor starts moving.

### Constants.java - SimulationConstants

Update simulation constants to match confirmed values:

```java
public static final double kHoodAngleDegrees = 60.0;        // confirmed 60 degrees
public static final double kFlywheelRadiusMeters = 0.0508;   // 2 inches in meters
public static final double kFuelMassKg = 0.227;              // 0.5 lbs in kg
public static final double kHighHubHeightMeters = 1.83;      // 72 inches in meters
```

### Validation on Robot

After updating constants from the calculator:
1. Measure actual windup time - compare to calculator estimate
2. Measure actual recovery time - compare to estimate
3. If predictions are too optimistic, reduce Efficiency % in calculator (try 85-90%)
4. Re-run calculator and update constants
5. Fine-tune kP on the robot if steady-state error exists
6. Validate distance-to-RPS mapping at multiple distances on the field

---

## Quick Reference: Motor Specs

| Motor | Free Speed (RPM) | Free Speed (RPS) | Stall Torque (Nm) | Stall Current (A) |
|-------|------------------|-------------------|--------------------|--------------------|
| **Kraken X60 (FOC)** | **6000** | **100.0** | **7.09** | **366** |
| Kraken X60 | 6000 | 100.0 | 7.09 | 233 |
| Kraken X44 (FOC) | 7530 | 125.5 | 3.44 | 366 |
| Kraken X44 | 7530 | 125.5 | 3.44 | 233 |
| NEO Vortex | 6784 | 113.1 | 3.60 | 211 |
| Falcon 500 | 6380 | 106.3 | 4.69 | 257 |
| NEO | 5880 | 98.0 | 3.36 | 181 |

Our flywheel uses **Kraken X60 (FOC)** (bolded above). Requires Phoenix Pro license.

Sources: [WCP Kraken X60](https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance),
[REV Motor Comparison](https://docs.revrobotics.com/brushless/neo/compare),
[Phoenix Pro Licensing](https://v6.docs.ctr-electronics.com/en/stable/docs/licensing/team-licensing.html)

---
