package frc.robot.subsystems.Vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Checkers;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

  private static final Vision INSTANCE = new Vision();

  private Vision() {}

  public static Vision getInstance() {
    return INSTANCE;
  }

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

  @Override
  public void initSendable(SendableBuilder builder) {

    // Send camera status to databoard
    for (Camera camera : getCameras()) {
      builder.addBooleanProperty(
          camera.getName() + " connection status", () -> camera.isConnected(), (v) -> {});
    }
  }
}
