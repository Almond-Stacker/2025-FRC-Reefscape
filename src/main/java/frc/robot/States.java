package frc.robot;

public class States {

    /*
    when ready just program it all into elevatorCommand
    where it will figure needed height divided between
    the two systems of the elevator. Mainly make a transition
    from relative rotations to a general unit like meters? or somethin
    public enum ElevatorStates {
        //overall height is primary + inner elevator heights
        PRIMARY_MIN(0),
        PRIMARY_MAX(101),
        INNER_MIN(0),
        INNER_MAX(5);

        public final double height;
        public final double angle;
        ElevatorStates(double height, double angle) {
            this.height = height;
            this.angle = angle;
        }

        ElevatorStates(double height) {
            this.height = height;
        }
    }
    */

    public enum PrimaryElevatorStates {
        // allow for easy changing of elevator states 
        HOME(0.57),
        L1(37),
        L2(99.5),
        L3(99.5),
        PRE_INTAKE(65),
        INTAKE(52),
        MIN(0),
        MAX(101);

        public final double height;

        PrimaryElevatorStates(double height) {
            this.height = height;
        }

        PrimaryElevatorStates() {
            this.height = 0; 
        }
    }

    public enum InnerElevatorStates {
        // allow for easy changing of elevator states 
        HOME(0.01),
        L1(2),
        L2(),
        L3(4.8),
        MIN(0),
        MAX(5);

        public final double height;

        InnerElevatorStates(double height) {
            this.height = height;
        }

        InnerElevatorStates() {
            this.height = 0; 
        }
    }

    public enum IntakeArmStates {
        // allow for easy changing of elevator states 
        HOME(210),
        INTAKE(60),
        STARTING_POSITION(240),
        L1(210),
        L2(210),
        L3(210),
        FEED_OUT(207),
        MIN(0),
        MAX(360);

        public final double angle;

        IntakeArmStates(double angle) {
            this.angle = angle;
        }

        IntakeArmStates() {
            this.angle = 0; 
        }
    }

    public enum SuckStates {
        // allow for easy changing of elevator states 
        STOP(0),
        INTAKE(1),
        FEED_OUT(-1);

        public final double speed;

        SuckStates(double speed) {
            this.speed = speed;
        }

        SuckStates() {
            this.speed = 0; 
        }
    }

    public enum ClimbStates {
        STOP(0),
        CLIMB(0.5),
        DROP(-0.5);

        public final double speed;

        ClimbStates(double speed) {
            this.speed = speed;
        }

        ClimbStates() {
            this.speed = 0;
        }
    }
}
