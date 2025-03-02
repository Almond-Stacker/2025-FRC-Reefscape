package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine StraightTest() {
        final AutoRoutine routine = m_factory.newRoutine("Straight Test");
        final AutoTrajectory simplePath = routine.trajectory("Straight Test");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    
    public AutoRoutine ShortTest() {
        final AutoRoutine routine = m_factory.newRoutine("Short Test");
        final AutoTrajectory simplePath = routine.trajectory("Short Test");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine CurveTest() {
        final AutoRoutine routine = m_factory.newRoutine("Curve Test");
        final AutoTrajectory simplePath = routine.trajectory("Curve Test");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine DiagonalTest() {
        final AutoRoutine routine = m_factory.newRoutine("Diagonal Test");
        final AutoTrajectory simplePath = routine.trajectory("Diagonal Test");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }
    
    public AutoRoutine GameAuto1() {
        final AutoRoutine routine = m_factory.newRoutine("Game Auto 1");
        final AutoTrajectory simplePath = routine.trajectory("Game Auto 1");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

 

}