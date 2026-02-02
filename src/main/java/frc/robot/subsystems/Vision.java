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

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;
  
  private AprilTagFieldLayout aprilTagFieldLayout;

  public Vision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

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
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if(visionEst.isEmpty() && !result.getTargets().isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }
    }
    visionEst.ifPresent(
        est -> {
          curStdDevs = getEstimationStdDevs();

          // Update estimator
          RobotContainer.driveTrain.updatePoseEstimate(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
    });
  }

  /** Returns the current estimation standard deviations (x, y, theta). */
  private Matrix<N3, N1> getEstimationStdDevs() {
    // TODO: Add distance based standard deviations
    if (curStdDevs != null) {
      return curStdDevs;
    }
    // Fallback defaults: fairly conservative uncertainty (meters, meters, radians)
    return VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
  }
}
