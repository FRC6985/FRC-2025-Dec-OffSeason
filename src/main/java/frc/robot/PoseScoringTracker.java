package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Superstructure;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class PoseScoringTracker {

  private static final int SCORING_POSES_COUNT = 24;
  private static final int SCORING_LEVELS_COUNT = 4;
  private static boolean[] probablyScoredPoses =
      new boolean[SCORING_POSES_COUNT * SCORING_LEVELS_COUNT];

  /**
   * Marks the closest fudged scoring pose as scored at the current scoring level. Called when the
   * robot successfully scores a piece.
   */
  public static void markPoseScored() {
    IndexedValue<Pose2d> closestPose =
        getClosestFudgedScoringPose(Robot.getInstance().getEstimatedPose());
    if (closestPose == null) {
      return;
    }

    int idx = closestPose.getKey();
    int levelIdx = Superstructure.getInstance().inputs.wantedScoringLevel.index;
    probablyScoredPoses[idx * SCORING_LEVELS_COUNT + levelIdx] = true;
  }

  /** Checks if a specific pose was already scored at the current scoring level. */
  public static boolean wasPoseScored(int idx) {
    int levelIdx = Superstructure.getInstance().inputs.wantedScoringLevel.index;
    return probablyScoredPoses[idx * SCORING_LEVELS_COUNT + levelIdx];
  }

  /** Resets all scored pose tracking (use at match start). */
  public static void resetScoredPoses() {
    probablyScoredPoses = new boolean[SCORING_POSES_COUNT * SCORING_LEVELS_COUNT];
  }

  /**
   * Calculates a score for how well the robot is aligned to a given pose. Lower score = better
   * alignment (closer translation + better rotation).
   */
  private static double score(Pose2d targetPose, Pose2d currentPose) {
    double translationDistance =
        targetPose.getTranslation().getDistance(currentPose.getTranslation());
    double rotationDifference =
        Math.abs((targetPose.getRotation().minus(currentPose.getRotation())).getRadians());

    return Constants.Extra.ALIGN_TRANSLATION_WEIGHT * translationDistance
        + Constants.Extra.ALIGN_ANGLE_WEIGHT * rotationDifference;
  }

  /**
   * Finds the closest coral/reef scoring pose that hasn't been used yet. Considers "badness" factor
   * for already scored positions.
   */
  public static boolean withinTolerance(Translation2d t) {
    return Robot.getInstance().getEstimatedPose().getTranslation().getDistance(t)
        < Constants.Extra.ALIGNMENT_TOLERANCE;
  }

  public static IndexedValue<Pose2d> getClosestFudgedScoringPose(Pose2d estimatedPose) {
    List<Pose2d> poses =
        Robot.getInstance().isOnRedSide()
            ? Constants.Field.redScoringPoses
            : Constants.Field.blueScoringPoses;

    if (poses == null || poses.isEmpty()) {
      return null;
    }

    Map.Entry<Integer, Pose2d> result =
        poses.stream()
            .filter(
                pose ->
                    pose.getTranslation().getDistance(estimatedPose.getTranslation())
                        < Constants.Extra.MAX_NODE_DISTANCE)
            .collect(Collectors.toMap(pose -> poses.indexOf(pose), pose -> pose))
            .entrySet()
            .stream()
            .min(
                (entry1, entry2) -> {
                  int idx1 = entry1.getKey();
                  int idx2 = entry2.getKey();
                  Pose2d pose1 = entry1.getValue();
                  Pose2d pose2 = entry2.getValue();

                  double fudge1 =
                      wasPoseScored(idx1) ? Constants.Extra.ALREADY_SCORED_BADNESS : 0.0;
                  double fudge2 =
                      wasPoseScored(idx2) ? Constants.Extra.ALREADY_SCORED_BADNESS : 0.0;

                  double score1 = fudge1 + score(pose1, estimatedPose);
                  double score2 = fudge2 + score(pose2, estimatedPose);

                  return Double.compare(score1, score2);
                })
            .orElse(null);

    if (result == null) {
      return null;
    }

    return new IndexedValue<>(result.getKey(), result.getValue());
  }

  public static Pose2d getClosestAlgaeGrabPose() {
    List<Pose2d> poses =
        Robot.getInstance().isRedAlliance()
            ? Constants.Field.redAlgaeGrabbingPose
            : Constants.Field.blueAlgaeGrabbingPose;

    // Filtreleme
    List<Pose2d> filtered =
        poses.stream()
            .filter(
                p ->
                    p.getTranslation()
                            .getDistance(Robot.getInstance().getEstimatedPose().getTranslation())
                        < Constants.Extra.MAX_NODE_DISTANCE)
            .toList();

    // Boş ise null döndür
    if (filtered.isEmpty()) return null;

    // En düşük score'a sahip olanı seç

    return filtered.stream()
        .min(
            (p1, p2) ->
                Double.compare(
                    score(p1, Robot.getInstance().getEstimatedPose()),
                    score(p2, Robot.getInstance().getEstimatedPose())))
        .orElse(null);
  }

  public static Pose2d getClosestTroughScoringPose() {
    List<Pose2d> poses =
        Robot.getInstance().isRedAlliance()
            ? Constants.Field.redTroughScoringPoses
            : Constants.Field.blueTroughScoringPoses;

    List<Pose2d> filtered =
        poses.stream()
            .filter(
                p ->
                    p.getTranslation()
                            .getDistance(Robot.getInstance().getEstimatedPose().getTranslation())
                        < Constants.Extra.MAX_NODE_DISTANCE)
            .toList();

    if (filtered.isEmpty()) return null;

    return filtered.stream()
        .min(
            (p1, p2) ->
                Double.compare(
                    score(p1, Robot.getInstance().getEstimatedPose()),
                    score(p2, Robot.getInstance().getEstimatedPose())))
        .orElse(null);
  }

  private static final double BARGE_SCORING_POSITION_TOLERANCE = 0.1;

  public static boolean atGoodScoringDistance() {
    if (!Robot.getInstance().isOnRedSide()) {
      // Blue alliance
      return Robot.getInstance().getEstimatedPose().getX()
              > Constants.Field.BLUE_BARGE_SCORING_X - BARGE_SCORING_POSITION_TOLERANCE
          && Robot.getInstance().getEstimatedPose().getX()
              < Constants.Field.BLUE_BARGE_SCORING_X + BARGE_SCORING_POSITION_TOLERANCE;
    } else {
      // Red alliance
      return Robot.getInstance().getEstimatedPose().getX()
              > Constants.Field.RED_BARGE_SCORING_X - BARGE_SCORING_POSITION_TOLERANCE
          && Robot.getInstance().getEstimatedPose().getX()
              < Constants.Field.RED_BARGE_SCORING_X + BARGE_SCORING_POSITION_TOLERANCE;
    }
  }

  /** Helper class for IndexedValue (like Kotlin's withIndex()). */
  public static class IndexedValue<T> {
    private final int key;
    private final T value;

    public IndexedValue(int key, T value) {
      this.key = key;
      this.value = value;
    }

    public int getKey() {
      return key;
    }

    public T getValue() {
      return value;
    }
  }
}
