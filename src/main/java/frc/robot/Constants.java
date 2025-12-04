package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Set;

public final class Constants {
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  };

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final class CanIds {
    public static final int IntakeCenteringMotor = 0;
    public static final int IntakeRollerMotor = 0;
    public static final int IntakePivotMotor = 0;
  }

  public static final class Field {
    public static final double FIELD_X_SIZE = 17.548249;
    public static final double FIELD_Y_SIZE = 8.051800;
  }

  public static final class Vision {
    // AprilTag Settings
    public static final AprilTagFields FieldType = AprilTagFields.k2025ReefscapeWelded;
    public static final Set<Integer> ignoredTagIds = Set.of(4, 5, 14, 15);

    // Camera names
    public static final String FrontRightName = "Front Right";
    public static final String FrontLeftName = "Front Left";
    public static final String BackRightName = "Back Right";
    public static final String BackLeftName = "Back Left";

    // Camera poses
    public static final Transform3d FRONT_RIGHT_TRANSFORM = new Transform3d(
        new Translation3d(-0.012552, -0.319809, 0.191168),
        new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-70.0)));
    public static final Transform3d BACK_RIGHT_TRANSFORM = new Transform3d(
        new Translation3d(-0.081165, -0.322330, 0.191168),
        new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-(180.0 - 55.0))));
    public static final Transform3d FRONT_LEFT_TRANSFORM = new Transform3d(
        new Translation3d(-0.012552, 0.319809, 0.191168),
        new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(70.0)));
    public static final Transform3d BACK_LEFT_TRANSFORM = new Transform3d(
        new Translation3d(-0.081165, 0.322330, 0.191168),
        new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(180.0 - 55.0)));
  }

  public final class Intake {
    public static enum RollerState {
      In(5, 4),
      SlowIn(2, 3),
      Out(-12, 0),
      TroughOut(3.25, 0),
      Off(0, 0);

      public final double rollingVoltage;
      public final double centeringVoltage;

      RollerState(double rollingVoltage, double centeringVoltage) {
        this.rollingVoltage = rollingVoltage;
        this.centeringVoltage = centeringVoltage;
      }
    }

    public static double SETPOINT_THRESHOLD = 0.1;

    public static enum PivotState {
      Up(0),
      Down(0),
      Trough(0);

      public final double pos;

      PivotState(double pos) {
        this.pos = pos;
      }
    }
  }

  public final class Arm {
    public static enum RollerState {
      In(0),
      AlgeaIn(0),
      AlgeaIdle(0),
      Out(0),
      SlowOut(0),
      AlgeaOut(0),
      Off(0);

      public final double rollingVoltage;

      RollerState(double rollingVoltage) {
        this.rollingVoltage = rollingVoltage;
      }
    }

    public static double SETPOINT_THRESHOLD = 0.1;

    public static enum PivotState {
      AlgeaInLeft(0),
      AlgeaInRight(0),
      CoralIn(0),
      AlgeaOutLeft(0),
      AlgeaOutRight(0),
      ScoreCoral(0),
      FinishScoreCoral(0),
      AboveScoreCoral(0),
      L4ScoreCoral(0),
      L4FinishScoreCoral(0),
      Start(0);

      public final double pos;

      PivotState(double pos) {
        this.pos = pos;
      }
    }
  }
}
