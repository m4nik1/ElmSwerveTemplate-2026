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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;
  
  private AprilTagFieldLayout aprilTagFieldLayout;

  private double lastSeenYawAlign = 0.0;
  private double alignDistance = 0.0;
  private boolean targetFound = false;

  public Vision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCamera);
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
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (var result : camera.getAllUnreadResults()) {
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if(visionEst.isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }
      
       if(!result.getTargets().isEmpty()) {
        for(var target: result.getTargets()) {
          if(target.getFiducialId() == 3 || target.getFiducialId() == 4) {
            lastSeenYawAlign = target.getYaw();
            targetFound = true;
            alignDistance = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
          }
          else {
            lastSeenYawAlign = 0.0;
            targetFound = false;
          }
        }
       }
       else {
        targetFound = false;
       }
    }
    
    visionEst.ifPresent(
        est -> {
          curStdDevs = getEstimationStdDevs();

          // Update estimator
          RobotContainer.driveTrain.updatePoseEstimate(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
        });

    Logger.recordOutput("targetFound "+ camera.getName(), targetFound());  
    Logger.recordOutput("Vision Yaw " + camera.getName(), getYawAlign());  
  }

  /** Returns the current estimation standard deviations (x, y, theta). */
  private Matrix<N3, N1> getEstimationStdDevs() {
    if (curStdDevs != null) {
      return curStdDevs;
    }
    
    // Fallback defaults: fairly conservative uncertainty (meters, meters, radians)
    return VecBuilder.fill(0.5, 0.5, 0.5);
  }
}
