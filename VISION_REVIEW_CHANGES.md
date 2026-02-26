# Vision.java Review (WPILib / PhotonVision) - Recommended Code Changes

This document captures recommended improvements to `/Users/cuescript/Documents/ElmSwerveTemplate-2026/src/main/java/frc/robot/subsystems/Vision.java` from an FRC WPILib + PhotonVision perspective.

## Summary of Why These Changes Matter

- Reduces pose estimate jumps/flips with single-tag fallback behavior.
- Makes target selection for alignment more stable and deterministic.
- Uses a better distance metric for drivetrain alignment (horizontal/planar distance).
- Removes `Vision` subsystem coupling to `RobotContainer`.
- Keeps vision measurement uncertainty behavior explicit and consistent.

---

## 1) `Vision.java`: Inject drivetrain callbacks instead of calling `RobotContainer` directly

**File:** `/Users/cuescript/Documents/ElmSwerveTemplate-2026/src/main/java/frc/robot/subsystems/Vision.java`

### Change imports

Remove the direct `RobotContainer` dependency and add `Supplier`.

```java
// REMOVE
import frc.robot.RobotContainer;

// ADD
import java.util.function.Supplier;
```

### Add callback fields

```java
private final EstimateConsumer estimateConsumer;
private final Supplier<Pose2d> referencePoseSupplier;
```

### Update constructor signature and configure PhotonPoseEstimator fallback

```java
public Vision(
    String name,
    Transform3d robotToCamera,
    EstimateConsumer estimateConsumer,
    Supplier<Pose2d> referencePoseSupplier) {
  camera = new PhotonCamera(name);

  AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  photonEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamera);

  // Important when only one tag is visible
  photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  this.estimateConsumer = estimateConsumer;
  this.referencePoseSupplier = referencePoseSupplier;
}
```

### Explanation

- Removes subsystem-to-`RobotContainer` static coupling.
- Allows `Vision` to be reused/tested more easily.
- Adds a proper fallback strategy when multi-tag solve is unavailable.

---

## 2) `Vision.java`: Make alignment target selection deterministic

**File:** `/Users/cuescript/Documents/ElmSwerveTemplate-2026/src/main/java/frc/robot/subsystems/Vision.java`

### Add helper methods for align-tag filtering and selection

```java
private static boolean isAlignTag(PhotonTrackedTarget target) {
  int id = target.getFiducialId();
  return id == 3 || id == 4;
}

private PhotonTrackedTarget pickBestAlignTarget(List<PhotonTrackedTarget> targets) {
  PhotonTrackedTarget best = null;
  for (var target : targets) {
    if (!isAlignTag(target)) continue;

    // Deterministic choice: largest image area is usually most stable
    if (best == null || target.getArea() > best.getArea()) {
      best = target;
    }
  }
  return best;
}
```

### Explanation

- Current behavior uses the first matching tag in the list, which can change frame-to-frame.
- Selecting by largest area tends to be more stable (closest/clearest target).

---

## 3) `Vision.java`: Use horizontal distance for alignment instead of full 3D norm

**File:** `/Users/cuescript/Documents/ElmSwerveTemplate-2026/src/main/java/frc/robot/subsystems/Vision.java`

### Replace `periodic()` alignment handling with deterministic target pick + planar distance

```java
@Override
public void periodic() {
  targetFound = false;
  alignDistance = Double.NaN;

  for (var result : camera.getAllUnreadResults()) {
    var bestAlignTarget = pickBestAlignTarget(result.getTargets());
    if (bestAlignTarget != null) {
      lastSeenYawAlign = bestAlignTarget.getYaw();
      targetFound = true;

      // Horizontal distance is better for drivetrain alignment than full 3D distance
      var t = bestAlignTarget.getBestCameraToTarget().getTranslation();
      alignDistance = Math.hypot(t.getX(), t.getY());
    }

    if (referencePoseSupplier != null) {
      photonEstimator.setReferencePose(referencePoseSupplier.get());
    }

    photonEstimator.update(result).ifPresent(est -> {
      if (est.targetsUsed.size() == 1 && est.targetsUsed.get(0).getPoseAmbiguity() > 0.2) {
        return;
      }

      var stdDevs = getEstimationStdDevs(est, getAverageDistance(est.targetsUsed));
      estimateConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
    });
  }
}
```

### Explanation

- `getNorm()` uses X/Y/Z and includes camera-vs-tag height difference.
- For drive alignment, the robot usually needs planar (field-plane) distance: `sqrt(x^2 + y^2)`.

---

## 4) `Vision.java`: Stop direct `RobotContainer.driveTrain` calls

**File:** `/Users/cuescript/Documents/ElmSwerveTemplate-2026/src/main/java/frc/robot/subsystems/Vision.java`

### Replace direct drivetrain call in pose update block

**Current:**
```java
RobotContainer.driveTrain.updatePoseEstimate(
    est.estimatedPose.toPose2d(),
    est.timestampSeconds,
    curStdDevs);
```

**Replace with:**
```java
estimateConsumer.accept(
    est.estimatedPose.toPose2d(),
    est.timestampSeconds,
    curStdDevs);
```

### Explanation

- Keeps `Vision` independent of robot container wiring.
- Moves system integration responsibility to `RobotContainer`, which is the right place.

---

## 5) `Vision.java`: Make `getEstimationStdDevs()` consistent (avoid `null` returns)

**File:** `/Users/cuescript/Documents/ElmSwerveTemplate-2026/src/main/java/frc/robot/subsystems/Vision.java`

### Suggested method implementation

```java
private Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose est, double averageDistance) {
  boolean isMultiTag = est.targetsUsed.size() > 1;
  double xyStdDev;

  if (!isMultiTag) {
    if (averageDistance < 1.0) xyStdDev = 0.35;
    else if (averageDistance <= 1.75) xyStdDev = 0.7;
    else if (averageDistance < 2.5) xyStdDev = 1.4;
    else return VecBuilder.fill(99, 99, 999999.0);
  } else {
    if (averageDistance < 1.0) xyStdDev = 0.10;
    else if (averageDistance < 2.0) xyStdDev = 0.20;
    else if (averageDistance < 4.0) xyStdDev = 0.40;
    else return VecBuilder.fill(99, 99, 999999.0);
  }

  // Keep heading effectively ignored unless/until multi-tag theta is trusted
  return VecBuilder.fill(xyStdDev, xyStdDev, 999999.0);
}
```

### Explanation

- Returning `null` forces extra branching and makes behavior less explicit.
- Returning a very large covariance is safer and simpler to reason about.

---

## 6) `RobotContainer.java`: Pass drivetrain callbacks into each `Vision` instance

**File:** `/Users/cuescript/Documents/ElmSwerveTemplate-2026/src/main/java/frc/robot/RobotContainer.java`

### Replace `visionRight` constructor call

```java
public static Vision visionRight = new Vision(
  "elm_right_cam",
  new Transform3d(
    Units.inchesToMeters(-3.5), Units.inchesToMeters(13), 0.0,
    new Rotation3d(0, Units.degreesToRadians(-20), 0)
  ),
  driveTrain::updatePoseEstimate,
  driveTrain::getPose
);
```

### Replace `visionLeft` constructor call

```java
public static Vision visionLeft = new Vision(
  "elm_left_cam",
  new Transform3d(
    Units.inchesToMeters(-3.5), Units.inchesToMeters(-13), 0.0,
    new Rotation3d(0, Rotation2d.fromDegrees(-20).getRadians(), 0)
  ),
  driveTrain::updatePoseEstimate,
  driveTrain::getPose
);
```

### Explanation

- This is the wiring point for subsystem collaboration.
- `Vision` now receives:
  - a pose update callback (`updatePoseEstimate`)
  - a reference pose supplier (`getPose`) for estimator stabilization

---

## 7) Optional but important: `DriveTrain.java` vision heading usage choice

**File:** `/Users/cuescript/Documents/ElmSwerveTemplate-2026/src/main/java/frc/robot/subsystems/DriveTrain.java`

### Current behavior (forces gyro heading, ignores vision theta)

```java
odom.addVisionMeasurement(new Pose2d(pose.getX(), pose.getY(), getYaw()), timestamp, stdDevs);
```

### Option A: Keep current behavior (valid if vision theta is intentionally ignored)

No change needed.

### Option B: Allow pose estimator to use vision heading

```java
odom.addVisionMeasurement(pose, timestamp, stdDevs);
```

### Explanation

- If `Vision.java` keeps theta std dev huge (`999999.0`), heading influence remains negligible anyway.
- If you later trust multi-tag heading and reduce theta std dev, use Option B.

---

## 8) Optional cleanup: remove unused imports in `Vision.java`

**File:** `/Users/cuescript/Documents/ElmSwerveTemplate-2026/src/main/java/frc/robot/subsystems/Vision.java`

Remove these if they become unused after changes:

```java
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
```

### Explanation

- Reduces noise and avoids confusion during debugging/review.

---

## Notes / Testing Suggestions (robot-side)

- Verify yaw alignment is stable when both target IDs (3 and 4) are visible.
- Verify `alignDistance` now matches driver expectations (planar distance).
- Check for fewer pose jumps when only one tag is visible.
- If pose still jumps, consider:
  - tightening ambiguity threshold for single-tag
  - using `setReferencePose(...)` every cycle (already included above)
  - gating updates by tag count / reprojection error (if available)

---
