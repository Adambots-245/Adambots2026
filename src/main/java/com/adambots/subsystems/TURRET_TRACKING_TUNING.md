# Turret Auto-Tracking Tuning Guide (V2 — PositionVoltage + Planetary)

All tracking constants are in `Constants.TurretTrackingConstants`.
PID gains are in `Constants.TurretConstants`.

## How It Works

The turret uses **PositionVoltage** with velocity feedforward (not Motion Magic).
Each camera frame (~13 Hz), the code computes a new setpoint and commands
the motor with both a position target and a velocity hint. The motor's PID
handles the fine control — the algorithm just decides WHERE to point.

**Tracking modes** (visible in AdvantageScope as `Turret/TrackAction`):
- **TRACK fresh** — camera sees hub, turret correcting toward it
- **DEADZONE** — turret is close enough to target, holding position
- **COAST stale** — camera frame expired, turret coasts on velocity FF
- **SWEEP** — hub not visible, scanning back and forth
- **JOG** — operator manually controlling with Xbox left stick

## Current Constants (simulation-optimized)

| Constant | Value | Units | Purpose |
|----------|-------|-------|---------|
| kTrackingToleranceDeg | 2.5 | deg | Dead zone — ignore offsets smaller than this |
| kCameraTrackingGain | 0.80 | 0-1 | EWMA: fraction of correction applied per fresh frame |
| kConvergeTimeSec | 0.15 | sec | Velocity FF time constant (approach speed) |
| kTrackingDebounceFrames | 0 | frames | Frames to wait before correcting (0 = immediate) |
| kCameraBrakeFrames | 2 | frames | Motor stop on SWEEP→CAMERA transition |
| kSweepWarmupFrames | 0 | frames | Delay before sweep starts (0 = immediate) |
| kAngularVelLeadTime | 0.05 | sec | Robot rotation feedforward anticipation |

## Tuning Order

### Step 1: Verify basic tracking
Enable auto-track (Button 5). Face the hub. The turret should lock on within 1 second.
- If turret doesn't move: check `Turret/AutoTrackEnabled` and `Turret/HubVisible` in AdvantageScope
- If turret sweeps but never locks: camera isn't detecting hub tags — check PhotonVision pipeline

### Step 2: Tune dead zone (`kTrackingToleranceDeg`)
Watch `Turret/CamYawInput` in AdvantageScope while the turret tracks.
- **If turret jitters/oscillates when locked**: Increase to 3.0-4.0
- **If turret sits with visible pointing error**: Decrease to 1.5-2.0
- **Sweet spot**: Just above the camera noise floor. Camera noise is ~0.5° std dev,
  so 2.0-3.0 is the ideal range.
- **Simulation found 2.5 optimal** — below 2° the turret chases noise

### Step 3: Tune tracking gain (`kCameraTrackingGain`)
This controls how much of the camera offset is corrected each fresh frame.
- 0.80 = correct 80% per frame → converges in 1-2 frames (~80ms)
- 0.50 = correct 50% per frame → converges in 3-4 frames (~240ms)
- **If turret overshoots and oscillates**: Decrease to 0.60-0.70
- **If turret tracks too slowly**: Increase to 0.85-0.90 (max 0.95)
- **Never use 1.0** — it amplifies camera noise directly to the setpoint

### Step 4: Tune velocity feedforward (`kConvergeTimeSec`)
Controls how aggressively the velocity FF drives the turret toward the setpoint.
Lower = faster approach but can overshoot through backlash.
- **If turret overshoots setpoint**: Increase to 0.20-0.25
- **If turret is sluggish reaching setpoint**: Decrease to 0.10-0.12
- **With the planetary gearbox**: 0.15 is optimal. The planetary's low backlash
  allows faster approach than without it (old value was 0.20).

### Step 5: Tune rotation compensation (`kAngularVelLeadTime`)
Watch tracking while spinning the robot. The turret should hold on the hub.
- **If turret lags behind during rotation**: Increase to 0.06-0.08
- **If turret overshoots during rotation**: Decrease to 0.03
- Camera latency is ~80ms, so 0.05-0.08 is the theoretical ideal range
- **Set to 0.0 to disable** for initial testing

### Step 6: Tune lock-on transition (`kCameraBrakeFrames`)
Controls how long the motor stops when transitioning from SWEEP to CAMERA.
- **If turret overshoots on first lock**: Increase to 3-5
- **If lock-on feels too slow**: Decrease to 1 or 0
- With planetary (low backdrive), 2 frames is sufficient

## PID Gains (TurretConstants)

These rarely need changing after initial tuning. Scaled for 4:1 planetary (44.44:1 total).

| Gain | Slot 0 (go-to-angle) | Slot 1 (tracking) | Purpose |
|------|---------------------|-------------------|---------|
| kP | 5.0 | 10.0 (kTurretTrackingP) | Position correction |
| kI | 0 | 0 | Not used |
| kD | 0.1 | 0.1 | Damping |
| kV | 0.025 | 0.025 | Velocity feedforward gain |
| kS | 0.10 | 0.10 | Static friction compensation |

- **Slot 0** is used for go-to-forward, hold, sweep-step, and limit clamp
- **Slot 1** is used for TRACK fresh with rotation compensation

**If turret buzzes at rest**: Decrease kS to 0.05
**If turret doesn't break through friction**: Increase kS to 0.15
**If tracking is sluggish**: Increase slot 1 kP to 15-20
**If tracking oscillates**: Decrease slot 1 kP to 5-8

## Diagnostics (AdvantageScope)

All signals are under `Turret/` in AdvantageScope:

| Signal | What to look for |
|--------|-----------------|
| TrackMode | Should be CAMERA when hub is visible |
| TrackAction | TRACK fresh = actively correcting, DEADZONE = on target |
| CurrentAngle | Should track Setpoint smoothly |
| Setpoint | Should converge toward hub position |
| CamYawInput | Camera offset — should be near 0 when locked |
| AngVelLead | Rotation compensation — should match robot turns |
| HubVisible / HubFresh | Detection quality — HubFresh should be >80% |
| JogInput | Xbox left stick X — should be 0 when not jogging |
| Debounce | Should be 0 with current settings |
| BrakeFrames | Should be 0 after initial lock-on |

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Turret doesn't move at all | AutoTrack not enabled | Press Button 5 |
| Turret sweeps but never locks | Camera not detecting hub tags | Check PV pipeline, kOdomMinTagAreaPercent |
| Turret locks briefly then sweeps | Hub detection flickering | Check camera exposure, reduce kCamCounterMax drain |
| Turret jitters when locked | Dead zone too small | Increase kTrackingToleranceDeg to 3.0 |
| Turret lags during rotation | Lead time too short | Increase kAngularVelLeadTime to 0.06-0.08 |
| Turret overshoots on lock | Gain too high or brake too low | Decrease kCameraTrackingGain, increase kCameraBrakeFrames |
| Turret moves in steps | TRACK bursts separated by DEADZONE | Normal for camera-rate control. Reduce tolerance if steps are too large |
| Turret screeches/buzzes | kS too high or kP oscillation | Decrease kS to 0.05, decrease kP |
| Turret drifts during DEADZONE | Rotation comp missing | Verify code has rotationCompVelDPS in DEADZONE/COAST blocks |
