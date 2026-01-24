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
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private static final int MIN_TAGS_FOR_MULTI = 2;
  private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.2;
  private static final double MAX_SINGLE_TAG_DISTANCE_M = 4.5;
  private static final double MAX_MULTI_TAG_DISTANCE_M = 6.0;
  private static final double MAX_POSE_JUMP_M = 1.5;
  private static final double FIELD_MARGIN_M = 0.5;

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;
  private EstimateConsumer estConsumer;
  
  private AprilTagFieldLayout aprilTagFieldLayout;

  public Vision(EstimateConsumer estConsumer, String name, Transform3d robotToCamera) {
    estConsumer = estConsumer;
    camera = new PhotonCamera(name);
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCamera);
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      visionEst = getValidEstimate(result);
    }

    visionEst.ifPresent(
        est -> {
          // Update estimator
          RobotContainer.driveTrain.updatePoseEstimate(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
        });
  }

  private Optional<EstimatedRobotPose> getValidEstimate(PhotonPipelineResult result) {
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> multiEst = Optional.empty();
    if (result.getTargets().size() >= MIN_TAGS_FOR_MULTI) {
      multiEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if (multiEst.isPresent() && isValidMultiTag(result, multiEst.get())) {
        curStdDevs = getEstimationStdDevs(result, true);
        return multiEst;
      }
    }

    Optional<EstimatedRobotPose> singleEst = photonEstimator.estimateLowestAmbiguityPose(result);
    if (singleEst.isPresent() && isValidSingleTag(result, singleEst.get())) {
      curStdDevs = getEstimationStdDevs(result, false);
      return singleEst;
    }

    return Optional.empty();
  }

  private boolean isValidMultiTag(PhotonPipelineResult result, EstimatedRobotPose est) {
    if (!isWithinField(est.estimatedPose.toPose2d())) {
      return false;
    }
    if (!isPoseJumpAllowed(est.estimatedPose.toPose2d())) {
      return false;
    }
    return getAverageTagDistance(result) <= MAX_MULTI_TAG_DISTANCE_M;
  }

  private boolean isValidSingleTag(PhotonPipelineResult result, EstimatedRobotPose est) {
    if (!isWithinField(est.estimatedPose.toPose2d())) {
      return false;
    }
    if (!isPoseJumpAllowed(est.estimatedPose.toPose2d())) {
      return false;
    }

    PhotonTrackedTarget bestTarget = result.getBestTarget();
    if (bestTarget == null) {
      return false;
    }
    if (bestTarget.getPoseAmbiguity() > MAX_SINGLE_TAG_AMBIGUITY) {
      return false;
    }
    return bestTarget.getBestCameraToTarget().getTranslation().getNorm() <= MAX_SINGLE_TAG_DISTANCE_M;
  }

  private boolean isWithinField(Pose2d pose) {
    double fieldLength = aprilTagFieldLayout.getFieldLength();
    double fieldWidth = aprilTagFieldLayout.getFieldWidth();

    double minX = -FIELD_MARGIN_M;
    double maxX = fieldLength + FIELD_MARGIN_M;
    double minY = -FIELD_MARGIN_M;
    double maxY = fieldWidth + FIELD_MARGIN_M;

    return pose.getX() >= minX && pose.getX() <= maxX && pose.getY() >= minY && pose.getY() <= maxY;
  }

  private boolean isPoseJumpAllowed(Pose2d pose) {
    Pose2d currentPose = RobotContainer.driveTrain.getPose();
    double delta = pose.getTranslation().getDistance(currentPose.getTranslation());
    return delta <= MAX_POSE_JUMP_M;
  }

  private double getAverageTagDistance(PhotonPipelineResult result) {
    if (result.getTargets().isEmpty()) {
      return 0.0;
    }
    double total = 0.0;
    for (PhotonTrackedTarget target : result.getTargets()) {
      total += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    return total / result.getTargets().size();
  }

  /** Returns the current estimation standard deviations (x, y, theta). */
  private Matrix<N3, N1> getEstimationStdDevs(PhotonPipelineResult result, boolean isMultiTag) {
    double baseXY = isMultiTag ? 0.25 : 0.5;
    double baseTheta = isMultiTag ? 0.35 : 0.7;

    double distanceScale = Math.max(1.0, getAverageTagDistance(result) / 3.0);
    double tagScale = 1.0 / Math.max(1, result.getTargets().size());
    double scale = Math.min(3.0, distanceScale / tagScale);

    return VecBuilder.fill(baseXY * scale, baseXY * scale, baseTheta * scale);
  }
}
