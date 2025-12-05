package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Set;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    public static TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Intake.GEAR_RATIO))
        .withSlot0(
            new Slot0Configs()
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKG(0.0)
                .withKP(160.0))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicJerk(2000.0)
                .withMotionMagicAcceleration(200.0)
                .withMotionMagicCruiseVelocity(2.0));

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

    public static double SETPOINT_THRESHOLD = Math.toRadians(7.0);
    public static double GEAR_RATIO = 1.0 / ((12.0 / 40.0) * (18.0 / 46.0) * (18.0 / 60.0) * (12.0 / 32.0));

    public static enum PivotState {
      Up(0.0),
      Down(Math.toRadians(126.0)),
      Trough(Math.toRadians(25.6));

      public final double angle;

      PivotState(double angle) {
        this.angle = angle;
      }
    }
  }

  public final class Arm {
    double Length = 0.0;// Arm part Length meter
    // private static PIVOT_ENCODER_RATIO = (36.0 / 18.0) * (36.0 / 18.0) * (60.0 /
    // 24.0) * (12.0 / 54.0)

    public static double PIVOT_GEAR_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (12.0 / 54.0);

    public static TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(1.0 / PIVOT_GEAR_RATIO))
        .withSlot0(
            new Slot0Configs()
                .withKS(0.0)
                .withKV(0.1)
                .withKA(0.0)
                .withKG(0.0)
                .withKP(80.0))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicJerk(9999.0)
                .withMotionMagicAcceleration(4.5)
                .withMotionMagicCruiseVelocity(2.0))
        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(70.0).withSupplyCurrentLimit(50.0));

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

      public final double angle;

      PivotState(double angle) {
        this.angle = angle;
      }
    }
  }

  public final class Elevator {

    public static final double SPOOL_RADIUS = Units.inchesToMeters(0.75);
    public static final double GEAR_RATIO = 4.0;

    public static TalonFXConfiguration motorConfig = new TalonFXConfiguration().withMotorOutput(
      new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake))
      .withFeedback(
          new FeedbackConfigs()
              .withSensorToMechanismRatio(GEAR_RATIO / (SPOOL_RADIUS * 2 * Math.PI)))
      .withSlot0(
          new Slot0Configs()
              .withKS(0.0)
              .withKV(0.0)
              .withKA(0.0)
              .withKG(0.37)
              .withKP(70.0))
      .withMotionMagic(
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(14.0)
              .withMotionMagicCruiseVelocity(3.0));


    // public static final double ZERO_VOLTAGE = -0.2;
    // public static final double ZERO_MIN_CURRENT = 1.7; // amps

    public static final double SETPOINT_THRESHOLD = 0.01;
    // public static final double LAZIER_SETPOINT_THRESHOLD = 0.03;

    // public static final double COLLISION_AVOIDANCE_MARGIN = 1.0;

    // public static final double MAX_EXTENSION = Units.inchesToMeters(55.0);

    // public static final double SAFE_HEIGHT = 0.837198 - 0.01;

   
  }

}
