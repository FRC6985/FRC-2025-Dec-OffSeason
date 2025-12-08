package frc.robot.subsystems.SuperStructure;

public class SuperstructureTransition {

    public interface Condition {
        boolean check();
    }

    public interface EnterFunction {
        void run();
    }

    public final SuperstructureState current;
    public final SuperstructureState next;
    public final EnterFunction enterFunction;
    public final Condition condition;

    public SuperstructureTransition(SuperstructureState current,
            SuperstructureState next,
            EnterFunction enterFunction,
            Condition condition) {
        this.current = current;
        this.next = next;
        this.enterFunction = enterFunction;
        this.condition = condition;
    }

    public SuperstructureTransition(SuperstructureState current,
            SuperstructureState next,
            Condition condition) {
        this(current, next, () -> {
        }, condition);
    }
}
