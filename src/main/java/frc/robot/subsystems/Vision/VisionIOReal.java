// Copyright 2025-2026 FRC 6985
// https://www.enkatech6985.com/
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.Vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.Constants;
import frc.robot.util.Checkers;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOReal implements VisionIO {
  private final Camera[] cameras = {
    new Camera(Constants.Vision.FrontRightName, Constants.Vision.FRONT_RIGHT_TRANSFORM),
    new Camera(Constants.Vision.FrontLeftName, Constants.Vision.FRONT_LEFT_TRANSFORM),
    new Camera(Constants.Vision.BackRightName, Constants.Vision.BACK_RIGHT_TRANSFORM),
    new Camera(Constants.Vision.BackLeftName, Constants.Vision.BACK_LEFT_TRANSFORM),
  };

  public final Camera[] getCameras() {
    return cameras;
  }

  public boolean allConnected() {
    boolean is = true;
    for (Camera camera : cameras) {
      if (!camera.isConnected()) {
        is = false;
        break;
      }
    }
    return is;
  }

  public boolean removeResult(PhotonPipelineResult res) {
    return res.getTargets().stream()
        .map(t -> t.getFiducialId())
        .anyMatch(Constants.Vision.ignoredTagIds::contains);
  }

  public void periodicAddMeasurements(SwerveDrivePoseEstimator poseEstimator) {
    for (Camera camera : cameras) {
      Stream<PhotonPipelineResult> results =
          camera.getAllUnreadResults().stream().filter(r -> removeResult(r));
      List<EstimatedRobotPose> estimatedPoses =
          results
              .map(r -> camera.poseEstimator.update(r).orElse(null))
              .filter(Objects::nonNull)
              .toList();

      for (EstimatedRobotPose pose : estimatedPoses) {
        if (Checkers.isInsideField(pose.estimatedPose.getTranslation().toTranslation2d())
            && (pose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                || (pose.strategy == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
                    && pose.targetsUsed.get(0).poseAmbiguity < 0.05
                    && pose.targetsUsed.get(0).area > 0.25))) {
          poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        }
      }
    }
  }
}
