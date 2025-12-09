package frc.robot.subsystems.SuperStructure;

import java.util.function.Supplier;

public class SuperstructureTransition {

    public final SuperstructureState current;
    public final SuperstructureState next;
    public final Runnable enterFunction;
    public final Supplier<Boolean> condition;

    public SuperstructureTransition(SuperstructureState current,
            SuperstructureState next,
            Runnable enterFunction,
            Supplier<Boolean> condition) {
        this.current = current;
        this.next = next;
        this.enterFunction = enterFunction;
        this.condition = condition;
    }

    public SuperstructureTransition(SuperstructureState current,
            SuperstructureState next,
            Supplier<Boolean> condition) {
        this(current, next, () -> {
        }, condition);
    }
}
