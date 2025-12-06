package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Utils;

import java.util.Set;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
                public static final int ArmRollerMotor = 0;
                public static final int ArmPivotMotor = 0;
                public static final int ElevatorMainMotor = 0;
                public static final int ElevatorFollowerMotor = 0;
        }

        public final class Field {

                public static final double FIELD_X_SIZE = 17.548249;
                public static final double FIELD_Y_SIZE = 8.051800;

                public static final double BLUE_BARGE_SCORING_X = 7.6;
                public static final double RED_BARGE_SCORING_X = FIELD_X_SIZE - BLUE_BARGE_SCORING_X;

                public static final double SAFE_WALL_DISTANCE = 1.0;

                /** Center of the blue reef */
                public static final Translation2d BLUE_REEF_CENTER = new Translation2d(Units.inchesToMeters(176.75),
                                Units.inchesToMeters(158.5));

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

                                for (int direction : new int[] { 1, -1 }) {
                                        for (int side : new int[] { 1, -1 }) {

                                                Pose2d base = new Pose2d(BLUE_REEF_CENTER, new Rotation2d());

                                                Transform2d t1 = new Transform2d(
                                                                new Translation2d(
                                                                                -ROBOT_REEF_CENTER_DISTANCE,
                                                                                0).rotateBy(angleRot),
                                                                new Rotation2d());

                                                Transform2d t2 = new Transform2d(
                                                                new Translation2d(
                                                                                direction * REEF_BRANCH_OFFSET_DISTANCE,
                                                                                0).rotateBy(anglePlus90),
                                                                angleRot);

                                                Transform2d t3 = new Transform2d(
                                                                new Translation2d(
                                                                                0.0,
                                                                                side * Arm.CORAL_CENTER_OFFSET),
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

                                for (int direction : new int[] { 1, -1 }) {

                                        Pose2d base = new Pose2d(BLUE_REEF_CENTER, new Rotation2d());

                                        Transform2d t1 = new Transform2d(
                                                        new Translation2d(
                                                                        -ROBOT_REEF_CENTER_DISTANCE,
                                                                        0).rotateBy(angleRot),
                                                        new Rotation2d());

                                        Transform2d t2 = new Transform2d(
                                                        new Translation2d(
                                                                        direction * TROUGH_OFFSET_DISTANCE,
                                                                        0).rotateBy(anglePlus90),
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

                                for (int side : new int[] { 1, -1 }) {

                                        Pose2d base = new Pose2d(BLUE_REEF_CENTER, new Rotation2d());

                                        Transform2d t1 = new Transform2d(
                                                        new Translation2d(
                                                                        -ROBOT_REEF_CENTER_DISTANCE + 0.05,
                                                                        0).rotateBy(angleRot),
                                                        angleRot);

                                        Transform2d t2 = new Transform2d(
                                                        new Translation2d(
                                                                        0.0,
                                                                        side * Arm.CORAL_CENTER_OFFSET),
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
                        for (Pose2d p : src)
                                mirrored.add(Utils.mirrorPose(p)); // uses your existing mirror method
                        return mirrored;
                }

                private Field() {
                }
        }

        // Arm angle (radians) -> Elevator height (meters)
        public static final List<double[]> ARM_ELEVATOR_PAIRS = Arrays.asList(
                        new double[] { Math.toRadians(7.0), 0.0 },
                        new double[] { Math.toRadians(18.775917), 0.044542 },
                        new double[] { Math.toRadians(26.241907), 0.068086 },
                        new double[] { Math.toRadians(34.899764), 0.100567 },
                        new double[] { Math.toRadians(44.171701), 0.137335 },
                        new double[] { Math.toRadians(55.042173), 0.184139 },
                        new double[] { Math.toRadians(68.556032), 0.253736 },
                        new double[] { Math.toRadians(180.0 - 96.880), 0.324134 },
                        new double[] { Math.toRadians(180.0 - 87.206214), 0.371332 },
                        new double[] { Math.toRadians(180.0 - 82.416832), 0.396130 },
                        new double[] { Math.toRadians(180.0 - 71.851303), 0.441318 },
                        new double[] { Math.toRadians(180.0 - 68.941494), 0.483176 },
                        new double[] { Math.toRadians(180.0 - 65.996990), 0.521174 },
                        new double[] { Math.toRadians(180.0 - 64.161654), 0.558276 },
                        new double[] { Math.toRadians(180.0 - 62.064537), 0.619367 },
                        new double[] { Math.toRadians(180.0 - 58.290246), 0.654566 },
                        new double[] { Math.toRadians(180.0 - 55.419787), 0.701323 - 0.0254 },
                        new double[] { Math.toRadians(180.0 - 47.462013), 0.750120 - 0.0254 },
                        new double[] { Math.toRadians(180.0 - 40.585547), 0.802161 - 0.0254 },
                        new double[] { Math.toRadians(180.0), Elevator.SAFE_HEIGHT });

        public static final List<double[]> ARM_INTERPOLATION_INTAKE_DOWN = Arrays.asList(
                        new double[] { Math.toRadians(88.0), 0.0 },
                        new double[] { Math.toRadians(103.296), 0.12 },
                        new double[] { Math.toRadians(113.75), 0.202 },
                        new double[] { Math.toRadians(117.77), 0.333 },
                        new double[] { Math.toRadians(131.77), 0.456 },
                        new double[] { Math.toRadians(137.593), 0.533 },
                        new double[] { Math.toRadians(180.0), 0.660 });

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

                public static double ZERO_VOLTAGE = -0.7;
                public static double ZERO_MIN_CURRENT = 20.0;
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
                                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(70.0)
                                                .withSupplyCurrentLimit(50.0));

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

                public static double CORAL_CENTER_OFFSET = Units.inchesToMeters(9.5);
        }

        public final class Elevator {

                public static final double SPOOL_RADIUS = Units.inchesToMeters(0.75);
                public static final double GEAR_RATIO = 3.6;

                public static TalonFXConfiguration motorConfig = new TalonFXConfiguration().withMotorOutput(
                                new MotorOutputConfigs()
                                                .withInverted(InvertedValue.Clockwise_Positive)
                                                .withNeutralMode(NeutralModeValue.Brake))
                                .withFeedback(
                                                new FeedbackConfigs()
                                                                .withSensorToMechanismRatio(GEAR_RATIO
                                                                                / (SPOOL_RADIUS * 2 * Math.PI)))
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

                public static final double ZERO_VOLTAGE = -0.2;
                public static final double ZERO_MIN_CURRENT = 1.7; // amps

                public static final double SETPOINT_THRESHOLD = 0.01;
                public static final double LAZIER_SETPOINT_THRESHOLD = 0.03;

                // public static final double COLLISION_AVOIDANCE_MARGIN = 1.0;

                public static final double MAX_EXTENSION = Units.inchesToMeters(55.0);

                public static final double SAFE_HEIGHT = 0.837198 - 0.01;

                public static enum State {
                        Down(0.0),
                        PreHandoff(Units.inchesToMeters(36.0)),
                        Handoff(Units.inchesToMeters(33.25)),
                        SourceIntake(Units.inchesToMeters(53.0)),
                        PreScore(Units.inchesToMeters(20.0)),
                        Trough(Units.inchesToMeters(38.0)),
                        L2(Units.inchesToMeters(15.0)),
                        L3(L2.rawExtension + Units.inchesToMeters(15.8701)),
                        L4(Units.inchesToMeters(54.5 - 0.125)),
                        Barge(Units.inchesToMeters(55.0 - 0.125)),
                        ScoreL4(L4.rawExtension - Units.inchesToMeters(1.0)),
                        ScoreL3(L3.rawExtension - Units.inchesToMeters(3.5)),
                        ScoreL2(L2.rawExtension - Units.inchesToMeters(3.5)),
                        PostL3(L2.rawExtension - Units.inchesToMeters(6.0)), // TODO: Tune
                        PostL2(L2.rawExtension - Units.inchesToMeters(3.5)), // TODO: Tune
                        AutoAlgae(Units.inchesToMeters(21.75)),
                        LowAlgae(Units.inchesToMeters(22.25)),
                        HighAlgae(LowAlgae.rawExtension + Units.inchesToMeters(15.8701)),
                        Processor(Units.inchesToMeters(20.0)),
                        AlgaeRest(Units.inchesToMeters(15.0)),
                        GroundAlgaeIntake(0.14),
                        PopsiclePickup(0.065);

                        public final double rawExtension;

                        State(double rawExtension) {
                                this.rawExtension = rawExtension;
                        }

                }

                public static enum AlgaeHeight

                {
                        High(Units.inchesToMeters(15.8701)),
                        Low(0.0);

                        public final double offset;

                        AlgaeHeight(double offset) {
                                this.offset = offset;
                        }
                }

        }

}
