# Turret Auto-Tracking Tuning Guide

All constants are in `Constants.TurretTrackingConstants` and `Constants.VisionConstants`.

## Vision Mode (`VisionConstants.kVisionMode`)

| Mode | Name     | Description |
|------|----------|-------------|
| 0    | Camera   | Direct camera pixel yaw only. Requires hub tags visible. |
| 1    | Pose     | Swerve pose bearing to hub center. No camera needed. |
| 2    | Hybrid   | Camera primary, pose fallback when camera loses tags. |
| 3    | Blended  | Weighted average of camera + pose (recommended). |

Start with **mode 3** for best reliability. Fall back to mode 0 if pose estimation is unreliable.

## Tracking Response

### `kCameraTrackingGain` (default: 0.15)
Proportional gain applied each cycle. Controls how aggressively the turret corrects toward the hub.

- **Increase** (toward 0.3) if turret tracks too slowly or lags behind robot movement.
- **Decrease** (toward 0.05) if turret oscillates or overshoots when locked.
- Safe range: 0.05 - 0.5.

### `kTrackingToleranceDeg` (default: 2.0)
Dead zone in degrees. Camera offsets smaller than this are ignored.

- **Increase** (3-5) if turret jitters when locked on target.
- **Decrease** (1.0) if turret isn't precise enough for consistent shots.
- Too low = chases camera noise. Too high = poor accuracy.

### `kAngularVelLeadTime` (default: 0.1s)
Feedforward time for robot rotation compensation. Turret leads the setpoint by `robotAngVel * leadTime`.

- **Increase** (0.15-0.2) if turret lags when robot spins fast.
- **Decrease** (0.05) if turret overshoots during rotation.
- Set to 0.0 to disable angular velocity compensation entirely.

## Lock-On Behavior

### `kCameraBrakeFrames` (default: 15)
Frames to stop the motor when transitioning from SWEEP to CAMERA mode. Lets turret decelerate from sweep before tracking starts.

- **Increase** (20-25) if turret overshoots on initial lock-on.
- **Decrease** (5-10) if lock-on feels sluggish.
- At 50Hz, 15 frames = 0.3 seconds.

### `kTrackingDebounceFrames` (default: 3)
Consecutive frames the camera offset must exceed the dead zone before corrections are applied. Filters single-frame jitter.

- **Increase** (5-10) if turret twitches from intermittent detections.
- **Decrease** (1-2) for faster response to real offset changes.

### `kSweepWarmupFrames` (default: 50)
Frames to hold at forward position before allowing sweep on startup. Gives vision time to initialize and detect hub tags.

- **Increase** if turret sweeps away before camera detects tags.
- **Decrease** if startup delay is too long.
- At 50Hz, 50 frames = 1.0 second.

## Vision Smoothing

### `kVisionAlpha` (default: 0.04)
Exponential Weighted Average filter on camera angle measurements. Lower = smoother but more lag.

- **Increase** (0.1-0.2) for faster response to angle changes.
- **Decrease** (0.01-0.03) for smoother tracking with less jitter.
- Warning: values above 0.2 can cause oscillation feedback loop.

### `kVisionBlendWeight` (default: 0.6)
Mode 3 only. Fraction of camera data in the blend.

- 1.0 = 100% camera (same as mode 0).
- 0.0 = 100% pose (same as mode 1).
- **Increase** if camera is more accurate than pose at your distances.
- **Decrease** if camera detection is intermittent.

### `kBlendDisagreementThreshold` (default: 20.0)
Mode 3 only. Max degrees of disagreement between camera and pose before falling back to camera-only.

- **Decrease** (10-15) if you see erratic tracking from bad blends.
- **Increase** (30+) if camera and pose are well-calibrated but noisy.

## Sweep

### `kScanStepDeg` (default: 4.5)
Degrees to advance per cycle during position-based sweep.

- **Decrease** (2-3) if camera misses hub tags during sweep.
- **Increase** (6-8) for faster reacquisition.

### `kScanMarginDeg` (default: 15.0)
Degrees from mechanical limits where sweep reverses direction.

- Prevents turret from hitting hard stops during sweep.

## Tuning Order

1. **Start with mode 3** and verify pose-based tracking works.
2. If turret oscillates when locked: increase `kTrackingToleranceDeg` or decrease `kCameraTrackingGain`.
3. If lock-on overshoots: increase `kCameraBrakeFrames`.
4. If turret lags during robot rotation: increase `kAngularVelLeadTime`.
5. Switch to **mode 0** and verify camera-only tracking.
6. If turret jitters in mode 0: increase `kTrackingDebounceFrames` or decrease `kVisionAlpha`.
7. Fine-tune `kVisionBlendWeight` in mode 3 based on field testing.
