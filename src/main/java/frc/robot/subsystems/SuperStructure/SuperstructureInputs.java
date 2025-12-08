package frc.robot.subsystems.SuperStructure;

public class SuperstructureInputs {

    public boolean wantExtend = false;
    public boolean wantGroundIntake = false;
    public boolean wantArmSourceIntake = false;
    public boolean wantSourceIntake = false;
    public boolean wantScore = false;

    public ScoringLevel wantedScoringLevel = ScoringLevel.L4;

    public boolean wantGetAlgae = false;
    public boolean wantDescoreAlgae = false;
    public boolean wantVerticalPickup = false;
    public boolean wantResetSuperstructure = false;
    public boolean wantScoreProcessor = false;
    public boolean wantAlgaeGroundIntake = false;
    public boolean wantPopsiclePickup = false;

    public SuperstructureInputs() {
    }

    @Override
    public String toString() {
        return "SuperstructureInputs{" +
                "wantExtend=" + wantExtend +
                ", wantGroundIntake=" + wantGroundIntake +
                ", wantArmSourceIntake=" + wantArmSourceIntake +
                ", wantSourceIntake=" + wantSourceIntake +
                ", wantScore=" + wantScore +
                ", wantedScoringLevel=" + wantedScoringLevel +
                ", wantGetAlgae=" + wantGetAlgae +
                ", wantDescoreAlgae=" + wantDescoreAlgae +
                ", wantVerticalPickup=" + wantVerticalPickup +
                ", wantResetSuperstructure=" + wantResetSuperstructure +
                ", wantScoreProcessor=" + wantScoreProcessor +
                ", wantAlgaeGroundIntake=" + wantAlgaeGroundIntake +
                ", wantPopsiclePickup=" + wantPopsiclePickup +
                '}';
    }

    public enum ScoringLevel {
        TROUGH,
        L2,
        L3,
        L4;

        public int index() {
            return switch (this) {
                case TROUGH -> 0;
                case L2 -> 1;
                case L3 -> 2;
                case L4 -> 3;
            };
        }
    }
}
