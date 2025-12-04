package frc.robot.subsystems.Vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  VisionIO io;

  public Vision(VisionIO io) {

    this.io = io;
  }

  public Vision() {}

  public void periodicAddMeasurements(SwerveDrivePoseEstimator estimator) {
    io.periodicAddMeasurements(estimator);
  }

  public boolean allConnected() {
    return io.allConnected();
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    // Send camera status to databoard
    for (Camera camera : io.getCameras()) {
      builder.addBooleanProperty(
          camera.getName() + " connection status", () -> camera.isConnected(), (v) -> {});
    }
  }
}
