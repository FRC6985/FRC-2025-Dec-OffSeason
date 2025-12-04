package frc.robot.subsystems.Vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  default Camera[] getCameras() {
    return new Camera[0];
  }

  default boolean allConnected() {
    return true;
  }

  default boolean removeResult(PhotonPipelineResult res) {
    return false;
  }

  default void periodicAddMeasurements(SwerveDrivePoseEstimator estimator) {}
}
