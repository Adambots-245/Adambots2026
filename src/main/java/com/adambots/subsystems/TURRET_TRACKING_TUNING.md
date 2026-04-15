# Turret Auto-Tracking Tuning Guide (Motion Magic + Planetary)

All tracking constants are in `Constants.TurretTrackingConstants`.
PID and Motion Magic profile are in `Constants.TurretConstants`.

## How It Works

The turret uses **Motion Magic** for all position commands. Each `setTurretAngle()`
call generates a trapezoidal velocity profile (accelerate → cruise → decelerate)
to reach the target. The tracking algorithm runs at ~50 Hz and decides WHERE to
point based on camera data.

**Key constraint**: Every new setpoint **restarts** the Motion Magic profile. During
active tracking, setpoints change every camera frame (~77ms). The strategy is to
use a HIGH tracking gain (0.90) so the turret converges in **1 frame**, then holds
in DEADZONE with minimal profile restarts. This produces smoother motion than
many small corrections.

**Tracking modes** (visible in AdvantageScope as `Turret/TrackAction`):
- **TRACK fresh** — camera sees hub, turret correcting (1-2 frames typically)
- **DEADZONE hold** — turret close enough to target, holding via Motion Magic
- **HOLD stale** — camera frame expired, holding last setpoint + angular vel lead
- **DEBOUNCE** — waiting for consistent offset before correcting
- **SWEEP** — hub not visible, scanning back and forth
- **JOG** — operator manually controlling with Xbox left stick

## Current Constants (simulation-optimized)

| Constant | Value | Units | Purpose |
|----------|-------|-------|---------|
| kTrackingToleranceDeg | 2.5 | deg | Dead zone — ignore offsets smaller than this |
| kCameraTrackingGain | 0.90 | 0-1 | Fraction of error corrected per fresh frame |
| kAngularVelLeadTime | 0.03 | sec | Robot rotation feedforward anticipation |
| kTrackingDebounceFrames | 0 | frames | Frames to wait before correcting (0 = immediate) |
| kCameraBrakeFrames | 2 | frames | Motor stop on SWEEP→CAMERA transition |
| kSweepWarmupFrames | 0 | frames | Delay before sweep starts (0 = immediate) |

### Motion Magic Profile

| Parameter | Value | Purpose |
|-----------|-------|---------|
| kTurretCruiseVelocity | 80 RPS | Max motor speed during profile |
| kTurretAcceleration | 400 RPS/s | Acceleration/deceleration rate |
| kTurretJerk | 0.0 | 0 = trapezoidal (no S-curve) |

Typical tracking correction is ~0.8° turret (0.1 motor rotations). At 80/400,
this is a triangular profile completing in ~30ms. The motor never reaches cruise
for these small moves — that's intentional and produces the smoothest motion.

## Tuning Order

### Step 1: Verify basic tracking
Enable auto-track (Button 5). Face the hub. The turret should lock on within 1 second.
- If turret doesn't move: check `Turret/AutoTrackEnabled` and `Turret/HubVisible`
- If turret sweeps but never locks: camera isn't detecting hub — check PV pipeline

### Step 2: Tune tracking gain (`kCameraTrackingGain`)
**This is the most important constant for Motion Magic.**

High gain = fewer TRACK frames = fewer profile restarts = smoother motion.
- 0.90 = correct 90% per frame → 1 frame to converge (optimal for MM)
- 0.70 = correct 70% per frame → 2-3 frames, more profile restarts, jerkier
- **If turret overshoots on each correction**: Decrease to 0.80-0.85
- **If turret tracks too slowly**: Increase to 0.95 (max practical value)
- **Simulation found 0.90 optimal** — 87% fewer trajectory restarts than 0.70

### Step 3: Tune dead zone (`kTrackingToleranceDeg`)
Watch `Turret/CamYawInput` while tracking.
- **If turret jitters when locked**: Increase to 3.0-3.5
- **If turret sits with visible pointing error**: Decrease to 2.0
- **Simulation found 2.5 optimal** — matches camera noise floor (~0.5° std dev)
- With 0.90 gain, the residual error entering DEADZONE is only ~0.25° (10% of
  the camera offset), so 2.5° tolerance catches nearly all corrections in 1 frame

### Step 4: Tune rotation compensation (`kAngularVelLeadTime`)
Watch tracking while spinning the robot.
- **If turret drifts during rotation**: Increase to 0.04-0.05
- **If turret overshoots or drifts during HOLD stale**: Decrease to 0.02
- **Simulation found 0.03 optimal** — higher values caused drift during HOLD stale
  frames (which are 79% of all frames with Motion Magic). The lead is applied every
  frame including HOLD stale, so excessive lead accumulates and creates drift.
- **Set to 0.0 to disable** for initial testing

### Step 5: Tune lock-on transition (`kCameraBrakeFrames`)
- **If turret overshoots on first lock**: Increase to 3-5
- **If lock-on feels too slow**: Decrease to 1 or 0
- With planetary (low backdrive), 2 frames is sufficient

### Step 6: Tune Motion Magic profile (if needed)
Only adjust if the turret feels too slow on large slews or too jerky on small corrections.
- **If turret is sluggish on large moves (>15°)**: Increase cruise to 100-120 RPS, accel to 500-600
- **If turret jerks on small corrections**: Decrease accel to 300-350 RPS/s
- **Sensitivity is LOW** — simulation showed <1% score difference across 80-150 RPS cruise range
- The profile mostly matters for SWEEP and go-to-forward, not tracking corrections

## PID Gains (TurretConstants)

Scaled for 4:1 planetary (44.44:1 total). Single slot (slot 0) for all Motion Magic commands.

| Gain | Value | Purpose |
|------|-------|---------|
| kP | 5.0 | Position correction (was 18 pre-planetary) |
| kI | 0 | Not used |
| kD | 0.1 | Damping |
| kV | 0.025 | Velocity feedforward (was 0.100 pre-planetary) |
| kS | 0.10 | Static friction compensation |

**If turret buzzes at rest**: Decrease kS to 0.05
**If turret doesn't break through friction**: Increase kS to 0.15
**If turret is sluggish**: Increase kP to 7-8
**If turret oscillates**: Decrease kP to 3-4

## Motion Magic vs PositionVoltage (V2 branch)

| Aspect | Motion Magic (this branch) | PositionVoltage (V2) |
|--------|---------------------------|---------------------|
| Smoothness | Better — profiled motion | Stepping at camera rate |
| Accuracy | 58.6% within 2° | 60.6% within 2° |
| Profile restarts | Yes — each setpoint restarts | No profiles |
| Best gain | 0.90 (minimize restarts) | 0.80 (noise filtering) |
| Best lead time | 0.03 (less drift in HOLD) | 0.05 (more aggressive) |
| Rotation comp in DEADZONE | Via angVelLead offset | Via velocity FF |

Motion Magic is **27% smoother** at the cost of 2% less accuracy. Both approaches
work well with the planetary gearbox.

## Diagnostics (AdvantageScope)

All signals under `Turret/`:

| Signal | What to look for |
|--------|-----------------|
| TrackMode | Should be CAMERA when hub visible |
| TrackAction | TRACK fresh should be brief bursts (1-2 frames), DEADZONE hold = on target |
| CurrentAngle | Should follow Setpoint smoothly |
| Setpoint | Should converge in 1 frame then hold steady |
| CamYawInput | Should be near 0 when locked |
| AngVelLead | Small values (±0.5°), larger during robot rotation |
| HubVisible / HubFresh | HubFresh >80% = good detection |
| JogInput | 0 when not jogging |

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Turret doesn't move | AutoTrack not enabled | Press Button 5 |
| Turret sweeps, never locks | Camera not detecting hub | Check PV pipeline |
| Turret locks briefly then sweeps | Hub detection flickering | Check exposure, kCamCounterMax |
| Turret jitters when locked | Dead zone too small | Increase kTrackingToleranceDeg to 3.0 |
| Turret screeches/stutters | Gain too low (too many MM restarts) | Increase kCameraTrackingGain to 0.90-0.95 |
| Turret lags during rotation | Lead time too short | Increase kAngularVelLeadTime to 0.04 |
| Turret drifts in HOLD stale | Lead time too high | Decrease kAngularVelLeadTime to 0.02 |
| Turret overshoots on lock | Brake frames too low | Increase kCameraBrakeFrames to 3-5 |
| Turret moves in large steps | Gain too low or tolerance too high | Increase gain or decrease tolerance |
| Turret buzzes at rest | kS too high | Decrease kS to 0.05 |
