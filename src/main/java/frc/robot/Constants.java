package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Utils;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

public final class Constants {
  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  };

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final class CanIds {

    public static int ARM_ENCODER = 0;
    public static int ARM_PIVOT_MOTOR = 0;
    public static int ARM_ROLLER_MOTOR = 0;
    public static int ELEVATOR_MAIN_MOTOR = 0;
    public static int ELEVATOR_FOLLOWER_MOTOR = 0;
    public static int INTAKE_PIVOT_MOTOR = 0;
    public static int INTAKE_ROLLER_MOTOR = 0;
    public static int INTAKE_CENTERING_MOTOR = 0;
  }

  public static final class DioIds {

    public static int INTAKE_LINEBREAK = 0;
  }

  public static final class Extra {
    public static double WHEEL_RADIUS = Units.inchesToMeters(3.9 / 2);
    public static double DRIVE_RATIO = 1.0 / ((16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
    public static double TURN_RATIO = 150.0 / 7.0;
    public static int INTAKE_LINEBREAK = 0;
    public static double ALIGNMENT_TOLERANCE = 0.04;
    public static double STARTING_TOLERANCE = 0.15;

    public static PIDController makeAlignTurnPID() {
      PIDController pidC = new PIDController(6.0, 0.0, 0.04);
      pidC.enableContinuousInput(-Math.PI, Math.PI);
      return pidC;
    }

    public static PIDController makeAlignDrivePID() {
      return new PIDController(5.0, 0.0, 0.01);
    }

    public static PIDController makeBargeAlignDrivePID() {
      return new PIDController(6.0, 0.0, 0.0);
    }

    public static double maxAlignTranslationSpeed = 1.5;
    public static double maxAlignRotationSpeed = 2.5;
    public static double maxBargeAlignTranslationSpeed = 1.5;
    public static double maxBargeAlignRotationSpeed = 1.5;
    public static double MAX_NODE_DISTANCE = 3.0; // meters
    // How fast the robot can move in a straight line (meters/sec).
    public static double MAX_VELOCITY = (5800.0 / 60) / DRIVE_RATIO * WHEEL_RADIUS * 2 * Math.PI;
    // How far the swerve modules are from (0,0).
    public static double XY_DISTANCE = Units.inchesToMeters(13.393747);
    // How fast the robot can rotate (radians/sec).
    public static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (XY_DISTANCE * Math.sqrt(2.0));

    public static double CHASSIS_RADIUS = (XY_DISTANCE * Math.sqrt(2.0));

    public static double ALIGN_ANGLE_WEIGHT = 2.7;
    public static double ALIGN_TRANSLATION_WEIGHT = 5.0;
    public static double ALREADY_SCORED_BADNESS =
        0.5 + Units.inchesToMeters(12.9375) * ALIGN_TRANSLATION_WEIGHT * (1 - 2 * 0.3);
  }

  public final class Field {

    public static final double FIELD_X_SIZE = 17.548249;
    public static final double FIELD_Y_SIZE = 8.051800;

    public static final double BLUE_BARGE_SCORING_X = 7.6;
    public static final double RED_BARGE_SCORING_X = FIELD_X_SIZE - BLUE_BARGE_SCORING_X;

    public static final double SAFE_WALL_DISTANCE = 1.0;

    /** Center of the blue reef */
    public static final Translation2d BLUE_REEF_CENTER =
        new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));

    /** Dist from robot center to reef center when aligned to a reef edge */
    public static final double ROBOT_REEF_CENTER_DISTANCE = Units.inchesToMeters(32.25 + 18.0);

    /** Branch offset */
    public static final double REEF_BRANCH_OFFSET_DISTANCE = Units.inchesToMeters(12.9375 / 2.0);

    public static final double TROUGH_OFFSET_DISTANCE = Units.inchesToMeters(14.5 / 2.0);

    // -----------------------------------------------------------
    // BLUE SCORING POSES
    // -----------------------------------------------------------
    public static final List<Pose2d> blueScoringPoses = generateBlueScoringPoses();

    private static List<Pose2d> generateBlueScoringPoses() {

      List<Pose2d> poses = new ArrayList<>();

      for (int angle = 0; angle < 360; angle += 60) {
        Rotation2d angleRot = new Rotation2d(Math.toRadians(angle));
        Rotation2d anglePlus90 = new Rotation2d(Math.toRadians(angle + 90));

        for (int direction : new int[] {1, -1}) {
          for (int side : new int[] {1, -1}) {

            Pose2d base = new Pose2d(BLUE_REEF_CENTER, new Rotation2d());

            Transform2d t1 =
                new Transform2d(
                    new Translation2d(-ROBOT_REEF_CENTER_DISTANCE, 0).rotateBy(angleRot),
                    new Rotation2d());

            Transform2d t2 =
                new Transform2d(
                    new Translation2d(direction * REEF_BRANCH_OFFSET_DISTANCE, 0)
                        .rotateBy(anglePlus90),
                    angleRot);

            Transform2d t3 =
                new Transform2d(
                    new Translation2d(0.0, side * Arm.CORAL_CENTER_OFFSET),
                    new Rotation2d(-side * Math.PI / 2.0));

            poses.add(base.plus(t1).plus(t2).plus(t3));
          }
        }
      }

      return poses;
    }

    // -----------------------------------------------------------
    // BLUE TROUGH SCORING POSES
    // -----------------------------------------------------------
    public static final List<Pose2d> blueTroughScoringPoses = generateBlueTroughPoses();

    private static List<Pose2d> generateBlueTroughPoses() {

      List<Pose2d> poses = new ArrayList<>();

      for (int angle = 0; angle < 360; angle += 60) {

        Rotation2d angleRot = new Rotation2d(Math.toRadians(angle));
        Rotation2d anglePlus90 = new Rotation2d(Math.toRadians(angle + 90));

        for (int direction : new int[] {1, -1}) {

          Pose2d base = new Pose2d(BLUE_REEF_CENTER, new Rotation2d());

          Transform2d t1 =
              new Transform2d(
                  new Translation2d(-ROBOT_REEF_CENTER_DISTANCE, 0).rotateBy(angleRot),
                  new Rotation2d());

          Transform2d t2 =
              new Transform2d(
                  new Translation2d(direction * TROUGH_OFFSET_DISTANCE, 0).rotateBy(anglePlus90),
                  angleRot);

          poses.add(base.plus(t1).plus(t2));
        }
      }

      return poses;
    }

    // -----------------------------------------------------------
    // BLUE ALGAE GRABBING POSES
    // -----------------------------------------------------------
    public static final List<Pose2d> blueAlgaeGrabbingPose = generateBlueAlgaeGrabPoses();

    private static List<Pose2d> generateBlueAlgaeGrabPoses() {

      List<Pose2d> poses = new ArrayList<>();

      for (int angle = 0; angle < 360; angle += 60) {

        Rotation2d angleRot = new Rotation2d(Math.toRadians(angle));

        for (int side : new int[] {1, -1}) {

          Pose2d base = new Pose2d(BLUE_REEF_CENTER, new Rotation2d());

          Transform2d t1 =
              new Transform2d(
                  new Translation2d(-ROBOT_REEF_CENTER_DISTANCE + 0.05, 0).rotateBy(angleRot),
                  angleRot);

          Transform2d t2 =
              new Transform2d(
                  new Translation2d(0.0, side * Arm.CORAL_CENTER_OFFSET),
                  new Rotation2d(-side * Math.PI / 2.0));

          poses.add(base.plus(t1).plus(t2));
        }
      }

      return poses;
    }

    // -----------------------------------------------------------
    // MIRRORED POSES (RED)
    // -----------------------------------------------------------

    public static final List<Pose2d> redScoringPoses = mirrorList(blueScoringPoses);

    public static final List<Pose2d> redTroughScoringPoses = mirrorList(blueTroughScoringPoses);

    public static final List<Pose2d> redAlgaeGrabbingPose = mirrorList(blueAlgaeGrabbingPose);

    // Mirror helper (equivalent to pose.mirror() in Kotlin utils)
    private static List<Pose2d> mirrorList(List<Pose2d> src) {
      List<Pose2d> mirrored = new ArrayList<>();
      for (Pose2d p : src) mirrored.add(Utils.mirrorPose(p)); // uses your existing mirror method
      return mirrored;
    }

    private Field() {}
  }

  public static final List<double[]> armElevatorPairs =
      Arrays.asList(
          new double[] {Math.toRadians(7.0), 0.0},
          new double[] {Math.toRadians(18.775917), 0.044542},
          new double[] {Math.toRadians(26.241907), 0.068086},
          new double[] {Math.toRadians(34.899764), 0.100567},
          new double[] {Math.toRadians(44.171701), 0.137335},
          new double[] {Math.toRadians(55.042173), 0.184139},
          new double[] {Math.toRadians(68.556032), 0.253736},
          new double[] {Math.toRadians(180.0 - 96.880), 0.324134},
          new double[] {Math.toRadians(180.0 - 87.206214), 0.371332},
          new double[] {Math.toRadians(180.0 - 82.416832), 0.396130},
          new double[] {Math.toRadians(180.0 - 71.851303), 0.441318},
          new double[] {Math.toRadians(180.0 - 68.941494), 0.483176},
          new double[] {Math.toRadians(180.0 - 65.996990), 0.521174},
          new double[] {Math.toRadians(180.0 - 64.161654), 0.558276},
          new double[] {Math.toRadians(180.0 - 62.064537), 0.619367},
          new double[] {Math.toRadians(180.0 - 58.290246), 0.654566},
          new double[] {Math.toRadians(180.0 - 55.419787), 0.701323 - 0.0254},
          new double[] {Math.toRadians(180.0 - 47.462013), 0.750120 - 0.0254},
          new double[] {Math.toRadians(180.0 - 40.585547), 0.802161 - 0.0254},
          new double[] {Math.toRadians(180.0), Elevator.SAFE_HEIGHT});

  public static final List<double[]> armInterpolationIntakeDown =
      Arrays.asList(
          new double[] {Math.toRadians(88.0), 0.0},
          new double[] {Math.toRadians(103.296), 0.12},
          new double[] {Math.toRadians(113.75), 0.202},
          new double[] {Math.toRadians(117.77), 0.333},
          new double[] {Math.toRadians(131.77), 0.456},
          new double[] {Math.toRadians(137.593), 0.533},
          new double[] {Math.toRadians(180.0), 0.660});

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
    public static final Transform3d FRONT_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.012552, -0.319809, 0.191168),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-70.0)));
    public static final Transform3d BACK_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.081165, -0.322330, 0.191168),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-(180.0 - 55.0))));
    public static final Transform3d FRONT_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.012552, 0.319809, 0.191168),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(70.0)));
    public static final Transform3d BACK_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.081165, 0.322330, 0.191168),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(180.0 - 55.0)));
  }

  public class Arm {
    public static final TalonFXConfiguration PIVOT_CONFIG;

    static {
      PIVOT_CONFIG = new TalonFXConfiguration();
      PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      PIVOT_CONFIG.Feedback.SensorToMechanismRatio = 1.0 / Arm.PIVOT_GEAR_RATIO;
      PIVOT_CONFIG.Slot0.kS = 0.0;
      PIVOT_CONFIG.Slot0.kV = 0.1;
      PIVOT_CONFIG.Slot0.kA = 0.0;
      PIVOT_CONFIG.Slot0.kG = 0.0;
      PIVOT_CONFIG.Slot0.kP = 80.0; // volts per rotation
      PIVOT_CONFIG.MotionMagic.MotionMagicJerk = 9999.0;
      PIVOT_CONFIG.MotionMagic.MotionMagicAcceleration = 4.5;
      PIVOT_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 2.0; // rps
      PIVOT_CONFIG.CurrentLimits.StatorCurrentLimit = 70.0;
      PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimit = 50.0;
    }

    public static final double POSITION_DEPENDENT_KG = 0.33;

    public static final double ALLOWED_OPERATING_RANGE_MIN = Math.toRadians(-350.0);
    public static final double ALLOWED_OPERATING_RANGE_MAX = Math.toRadians(350.0);

    public static final double PIVOT_ENCODER_RATIO =
        (36.0 / 18.0) * (36.0 / 18.0) * (60.0 / 24.0) * (12.0 / 54.0);

    public static final double PIVOT_GEAR_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (12.0 / 54.0);

    public static final double PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS = -0.053;

    public static final double CORAL_CENTER_OFFSET = Units.inchesToMeters(9.5);
    public static final double SAFE_DISTANCE_FROM_REEF_CENTER = Units.inchesToMeters(70.0);
    public static final double SAFE_PLACEMENT_DISTANCE = Units.inchesToMeters(60.0);
    public static final double SAFE_BARGE_DISTANCE = Units.inchesToMeters(50.0);
    public static final double SAFE_INSIDE_ROBOT_ANGLE = Math.toRadians(40.0);

    public static final double SETPOINT_THRESHOLD = 0.1;
    public static final double CURRENT_DRAW = 20.0;
    public static final double IDLE_CURRENT_DRAW = 10.0;
  }

  public class Elevator {
    public static final double SPOOL_RADIUS = Units.inchesToMeters(0.75);
    public static final double GEAR_RATIO = 3.6;

    public static final double ZERO_VOLTAGE = -0.2;
    public static final double ZERO_MIN_CURRENT = 1.7; // amps

    public static final double SETPOINT_THRESHOLD = 0.01;
    public static final double LAZIER_SETPOINT_THRESHOLD = 0.03;

    public static final double COLLISION_AVOIDANCE_MARGIN = 1.0;

    public static final double MAX_EXTENSION = Units.inchesToMeters(55.0);

    public static final double SAFE_HEIGHT = 0.837198 - 0.01;

    public static final TalonFXConfiguration MOTOR_CONFIG;

    static {
      MOTOR_CONFIG = new TalonFXConfiguration();
      MOTOR_CONFIG.Feedback.SensorToMechanismRatio = GEAR_RATIO / (SPOOL_RADIUS * 2 * Math.PI);
      MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      MOTOR_CONFIG.Slot0.kS = 0.0;
      MOTOR_CONFIG.Slot0.kV = 0.0;
      MOTOR_CONFIG.Slot0.kA = 0.0;
      MOTOR_CONFIG.Slot0.kG = 0.37;
      MOTOR_CONFIG.Slot0.kP = 70.0;
      // MOTOR_CONFIG.MotionMagic.MotionMagicJerk = 9999.0;
      MOTOR_CONFIG.MotionMagic.MotionMagicAcceleration = 14.0;
      MOTOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 3.0;
    }
  }

  public class Intake {
    public static final TalonFXConfiguration PIVOT_CONFIG;

    static {
      PIVOT_CONFIG = new TalonFXConfiguration();
      PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      PIVOT_CONFIG.Feedback.SensorToMechanismRatio = Intake.GEAR_RATIO;
      PIVOT_CONFIG.Slot0.kS = 0.0;
      PIVOT_CONFIG.Slot0.kV = 0.0;
      PIVOT_CONFIG.Slot0.kA = 0.0;
      PIVOT_CONFIG.Slot0.kG = 0.0;
      PIVOT_CONFIG.Slot0.kP = 160.0;
      PIVOT_CONFIG.MotionMagic.MotionMagicJerk = 2000.0;
      PIVOT_CONFIG.MotionMagic.MotionMagicAcceleration = 200.0;
      PIVOT_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 2.0;
      PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    private static final double GEAR_RATIO =
        1.0 / ((12.0 / 40.0) * (18.0 / 46.0) * (18.0 / 60.0) * (12.0 / 32.0));
    public static final double ZERO_VOLTAGE = -0.7;
    public static final double ZERO_MIN_CURRENT = 20.0; // amps

    public static final double SETPOINT_THRESHOLD = Math.toRadians(7.0);
    public static final double ULTRASONIC_SENSOR_THRESHOLD = 0.02;
  }
}
