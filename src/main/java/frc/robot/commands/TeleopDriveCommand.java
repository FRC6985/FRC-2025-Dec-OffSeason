package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Controls.DriveInputs;
import frc.robot.PoseScoringTracker;
import frc.robot.Robot;
import frc.robot.util.Utils;
import java.util.function.Supplier;

public class TeleopDriveCommand extends Command {

  private final Supplier<DriveInputs> driveInputsSupplier;

  private Controls.AlignMode previousAlignMode = Controls.AlignMode.None;

  private final PIDController xPID = Constants.Extra.makeAlignDrivePID();
  private final PIDController yPID = Constants.Extra.makeAlignDrivePID();
  private final PIDController turnPID = Constants.Extra.makeAlignTurnPID();

  public TeleopDriveCommand(Supplier<DriveInputs> driveInputsSupplier) {
    this.driveInputsSupplier = driveInputsSupplier;
  }

  @Override
  public void execute() {
    DriveInputs inputs = driveInputsSupplier.get();

    // Red alliance flip
    if (Robot.getInstance().isRedAlliance()) inputs = inputs.redFlipped();

    // Disable align if driver is moving joystick
    if (inputs.isNonZero() && !Robot.getInstance().isAutonomous()) {
      inputs =
          new DriveInputs(
              inputs.forward,
              inputs.left,
              inputs.rotation,
              inputs.deadzone,
              Controls.AlignMode.None);
    }

    // Reset PID when switching align mode
    if (inputs.alignMode != previousAlignMode) {
      xPID.reset();
      yPID.reset();
      turnPID.reset();
    }
    previousAlignMode = inputs.alignMode;

    Robot.getInstance().setIsAligned(false);

    ChassisSpeeds speeds;

    if (inputs.alignMode == Controls.AlignMode.None) {

      speeds = chassisSpeedsFromDriveInputs(inputs);

    } else if (inputs.alignMode == Controls.AlignMode.BargeAlign) {

      double targetX =
          Robot.getInstance().isOnRedSide()
              ? Constants.Field.FIELD_X_SIZE - Constants.Field.BLUE_BARGE_SCORING_X
              : Constants.Field.BLUE_BARGE_SCORING_X;

      double xSpeed =
          fixBargeTranslationInput(
              xPID.calculate(Robot.getInstance().getEstimatedPose().getX(), targetX));

      double targetRot =
          Math.abs(Math.PI / 2 - Robot.getInstance().getEstimatedPose().getRotation().getRadians())
                  < Math.abs(
                      -Math.PI / 2
                          - Robot.getInstance().getEstimatedPose().getRotation().getRadians())
              ? Math.PI / 2
              : -Math.PI / 2;

      double rotSpeed =
          fixRotationInput(
              turnPID.calculate(
                  Robot.getInstance().getEstimatedPose().getRotation().getRadians(), targetRot));

      speeds = new ChassisSpeeds(xSpeed, 0.0, rotSpeed);

    } else {

      Pose2d pose = null;

      switch (inputs.alignMode) {
        case TroughAlign:
          pose = PoseScoringTracker.getClosestTroughScoringPose();
          break;
        case ReefAlign:
          var fudged =
              PoseScoringTracker.getClosestFudgedScoringPose(
                  Robot.getInstance().getEstimatedPose());
          pose = (fudged != null) ? fudged.getValue() : null;
          break;
        case AlgaeAlign:
          pose = PoseScoringTracker.getClosestAlgaeGrabPose();
          break;
        default:
          throw new IllegalStateException("Unexpected align mode");
      }

      if (pose == null) {
        speeds = chassisSpeedsFromDriveInputs(inputs);
      } else {
        Robot.getInstance().setIsAligned(PoseScoringTracker.withinTolerance(pose.getTranslation()));

        double xSpeed =
            fixTranslationInput(
                xPID.calculate(Robot.getInstance().getEstimatedPose().getX(), pose.getX()));

        double ySpeed =
            fixTranslationInput(
                yPID.calculate(Robot.getInstance().getEstimatedPose().getY(), pose.getY()));

        double rotSpeed =
            fixRotationInput(
                turnPID.calculate(
                    Robot.getInstance().getEstimatedPose().getRotation().getRadians(),
                    pose.getRotation().getRadians()));

        speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
      }
    }

    Robot.getInstance().drive.driveFieldRelative(speeds);
  }

  // -------------------- Helper functions --------------------

  private double fixRotationInput(double n) {
    return Utils.unclampedDeadzone(
        clamp(n, -Constants.Extra.maxAlignRotationSpeed, Constants.Extra.maxAlignRotationSpeed),
        0.03);
  }

  private double fixTranslationInput(double n) {
    return Utils.unclampedDeadzone(
        clamp(
            n, -Constants.Extra.maxAlignTranslationSpeed, Constants.Extra.maxAlignTranslationSpeed),
        0.03);
  }

  private double fixBargeTranslationInput(double n) {
    return Utils.unclampedDeadzone(
        clamp(
            n,
            -Constants.Extra.maxBargeAlignTranslationSpeed,
            Constants.Extra.maxBargeAlignTranslationSpeed),
        0.03);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  public ChassisSpeeds chassisSpeedsFromDriveInputs(DriveInputs inputs) {
    double x = inputs.forward;
    double y = inputs.left;
    double rotation = inputs.rotation;

    double theta = Math.atan2(y, x);
    double r = Math.hypot(x, y);

    r = Utils.deadZone(r, inputs.deadzone);
    rotation = Utils.deadZone(rotation, inputs.deadzone);

    r = r * r; // square input
    rotation = rotation * rotation * Math.signum(rotation);

    double actualX = r * Math.cos(theta) * Constants.Extra.MAX_VELOCITY;
    double actualY = r * Math.sin(theta) * Constants.Extra.MAX_VELOCITY;
    double actualRot = rotation * Constants.Extra.MAX_ANGULAR_VELOCITY;

    return new ChassisSpeeds(actualX, actualY, actualRot);
  }
}
