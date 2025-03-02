package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto1() {
        final AutoRoutine routine = m_factory.newRoutine("Straight Test");
        final AutoTrajectory simplePath = routine.trajectory("Straight Test");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    
    public AutoRoutine simplePathAuto2() {
        final AutoRoutine routine = m_factory.newRoutine("New Path");
        final AutoTrajectory simplePath = routine.trajectory("New Path");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine simplePathAuto3() {
        final AutoRoutine routine = m_factory.newRoutine("New Path (1)");
        final AutoTrajectory simplePath = routine.trajectory("New Path (1)");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    
    public AutoRoutine simplePathAuto4() {
        final AutoRoutine routine = m_factory.newRoutine("New Path (2)");
        final AutoTrajectory simplePath = routine.trajectory("New Path (2)");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

 


}