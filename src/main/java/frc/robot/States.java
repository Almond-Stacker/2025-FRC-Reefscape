package frc.robot;

public class States {
    public enum ElevatorStates {
        MIN(0, 3.211, 55),
        MAX(112 , 26,220),
        STARTING_POSITION(0.05, 23,71),
        PRE_INTAKE(63.55, 24, 90),
        INTAKE(63.55, 22, 70),
        L1(0.5,6,210),
        L2(1,7.416,203),
        L3(23.9, 23, 200),
        L4(108.5, 23, 210);
        public final double primaryHeight; 
        public final double innerHeight;
        
        public final double armAngle;

        ElevatorStates(double primaryHeight, double innerHeight, double intakeArmAngle) {
            this.primaryHeight = primaryHeight;
            this.innerHeight = innerHeight;
            this.armAngle = intakeArmAngle;
        }
    }

    public enum IndexStates {
        INTAKE(1),
        OUTTAKE(-1),
        STOP(0);

        public final double speed;

        IndexStates(double speed) {
            this.speed = speed;
        }
    }

    public enum ClimbStates {
        CLIMB(1),
        STOP(0),
        DROP(-1);

        public final double speed;

        ClimbStates(double speed) {
            this.speed = speed;
        }
    }
}
