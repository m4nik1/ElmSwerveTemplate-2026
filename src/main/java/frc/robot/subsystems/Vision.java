// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  private AprilTagFieldLayout aprilTagFieldLayout;

  private double lastSeenYawAlign = 0.0;
  private double alignDistance = 0.0;

  public Vision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCamera);
  }

  // Function gets the angle from the vision targets
  // From the hub
  public double getYawAlign() {
    return lastSeenYawAlign;
  }

  public double getAlignDistance() {
    return alignDistance;
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  private double getAverageDistance(List<PhotonTrackedTarget> targets) {
    double totalDistance = 0.0;
    int count = 0;

    // Go through each target for that pose
    //
    for (var target : targets) {
      // Get the transform from the camera to the tag
      var cameraToTarget = target.getBestCameraToTarget().getTranslation();

      // Calculate the 3D distance using the translation component
      double distance = Math.hypot(cameraToTarget.getX(), cameraToTarget.getY());

      totalDistance += distance;
      count++;
    }

    if (count == 0)
      return 0.0;

    return totalDistance / count;
  }

  public double getDistanceClosestCamera(List<PhotonTrackedTarget> targets, EstimatedRobotPose est) {
    return targets.stream()
        .mapToDouble(trackedTarget -> est.estimatedPose.toPose2d()
            .relativeTo(Constants.fieldLayout.getTagPose(trackedTarget.getFiducialId()).get().toPose2d())
            .getTranslation().getNorm())
        .min().orElse(1000);
  }

  private static boolean isAlignTag(PhotonTrackedTarget target) {
    int id = target.getFiducialId();
    return id == 25 || id == 26;
  }

  @Override
  public void periodic() {
    alignDistance = Double.NaN;

    for (var result : camera.getAllUnreadResults()) {
      if (!result.getTargets().isEmpty()) {
        for (var target : result.getTargets()) {
          if (isAlignTag(target)) {
            lastSeenYawAlign = target.getYaw();
            alignDistance = target.getBestCameraToTarget().getTranslation().getNorm();
            break;
          }
        }
      }


      if(camera.getName() == "elm_left_cam") {
        photonEstimator.update(result).ifPresent(est -> {
          if (est.targetsUsed.size() == 1 && est.targetsUsed.get(0).getPoseAmbiguity() > 0.15) {
            // If we only have one target and its pose ambiguity is high, skip updating the
            // pose
            return;
          }
          // getEstimationStdDevs(est, getAverageDistance(est.targetsUsed));
          var curStdDevs = getEstimationStdDevs(est, getAverageDistance(est.targetsUsed));

          if (curStdDevs != null) {
            RobotContainer.driveTrain.updatePoseEstimate(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
          }
        });
      }
    }
    // Logger.recordOutput("targetFound " + camera.getName(), targetFound());
    Logger.recordOutput("Vision Yaw " + camera.getName(), getYawAlign());
  }

  /** Returns the current estimation standard deviations (x, y, theta). */
  private Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose est, double averageDistance) {
    // If dont have a good distance, use conservative defaults
    double stdDeviation = 2;
    boolean isMultiTag = est.targetsUsed.size() > 1;

    Logger.recordOutput("Auto/alignDistance", getDistanceClosestCamera(est.targetsUsed, est));
    Logger.recordOutput("Auto/AverageDistance", averageDistance);
    Logger.recordOutput("Is multitag", isMultiTag);

    if (!isMultiTag) {
      if (averageDistance < 1)
        stdDeviation = 0.22;
      else if (averageDistance <= 1.75)
        stdDeviation = 0.7;
      else if (averageDistance < 2.5)
        stdDeviation = 1.4;
      else
        return VecBuilder.fill(99, 99, 99);
    } else {
      if (averageDistance < 1)
        stdDeviation = 0.15;
      else if (averageDistance < 2)
        stdDeviation = 0.25;
      else if (averageDistance < 4)
        stdDeviation = 0.45;
      else
        return VecBuilder.fill(99, 99, 99);
    }

    // Fallback defaults: fairly conservative uncertainty (meters, meters, radians)
    return VecBuilder.fill(stdDeviation, stdDeviation, 999999.0);
  }
}
