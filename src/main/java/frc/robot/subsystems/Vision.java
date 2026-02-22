// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;

  private AprilTagFieldLayout aprilTagFieldLayout;

  private double lastSeenYawAlign = 0.0;
  private double alignDistance = 0.0;
  private double standardDevDistance = 0.0;
  private boolean targetFound = false;
  private boolean isMultiTag = false;

  public Vision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCamera);
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }

  // Function gets the angle from the vision targets
  // From the hub
  public double getYawAlign() {
    return lastSeenYawAlign;
  }

  public double getAlignDistance() {
    return alignDistance;
  }

  public boolean targetFound() {
    return targetFound;
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  @Override
  public void periodic() {
    targetFound = false;
    for (var result : camera.getAllUnreadResults()) {
      if (!result.getTargets().isEmpty()) {
        if (result.getBestTarget() != null) {
          standardDevDistance = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        }
        isMultiTag = result.multitagResult != null;
        // Setting target found to false
        targetFound = false;

        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 3 || target.getFiducialId() == 4) {
            lastSeenYawAlign = target.getYaw();
            targetFound = true;
            alignDistance = target.getBestCameraToTarget().getTranslation().getNorm();

            break;
          } 
        }
      } else {
        targetFound = false;
      }

      photonEstimator.update(result).ifPresent(est -> {
        curStdDevs = getEstimationStdDevs();
        if(curStdDevs != null) {
          RobotContainer.driveTrain.updatePoseEstimate(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
        }
      });
    }
    // Logger.recordOutput("targetFound " + camera.getName(), targetFound());
    // Logger.recordOutput("Vision Yaw " + camera.getName(), getYawAlign());
  }

  /** Returns the current estimation standard deviations (x, y, theta). */
  private Matrix<N3, N1> getEstimationStdDevs() {
    // If dont have a good distance, use conservative defaults
    double stdDeviation = 2;

    if (!isMultiTag) {
      if(standardDevDistance < 1) {
        stdDeviation = 0.35;
      } else if (standardDevDistance <= 1.75) {
        stdDeviation = 0.7;
      } else if (standardDevDistance < 2.5) {
        stdDeviation = 1.4;
      } else if (standardDevDistance < 3) {
        stdDeviation = 2.8;
      } else {
        return null;
      }
    } else {
      if (standardDevDistance < 1) {
        stdDeviation = 0.07;
      } else if (standardDevDistance < 2) {
        stdDeviation = 0.11;
      } else if (standardDevDistance < 3) {
        stdDeviation = 0.16;
      } else if (standardDevDistance < 4) {
        stdDeviation = 0.2;
      } else {
        return null;
      }
    }

    // Fallback defaults: fairly conservative uncertainty (meters, meters, radians)
    return VecBuilder.fill(stdDeviation, stdDeviation, Units.degreesToRadians(100));
  }
}
