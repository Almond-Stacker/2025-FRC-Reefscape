package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class States {

    public enum ElevatorStates {
        //tune
        HOME_ABS(0.57, 0.1004, 210),
        MAX_ABS(101, 0.8, 255),
        MIN_ABS(0, -2.7, 70),
        HOME_REL(0.5, 210),
        L2_REL(.9, 210),
        L3_REL(1.5, 190),
        L4_REL(1.8, 180);

        public final double primaryHeight;
        public final double innerHeight;
        public final double relTotalHeight;
        public final double angle;

        ElevatorStates(double primaryHeight, double innerHeight, double angle) {
            this.primaryHeight = primaryHeight;
            this.innerHeight = innerHeight;
            this.angle = angle;

            this.relTotalHeight = 0;
            SmartDashboard.putNumber("Relative Goal", relTotalHeight);

        }

        ElevatorStates(double relTotalHeight, double angle) {
            this.relTotalHeight = relTotalHeight;
            this.angle = angle;

            this.primaryHeight = 0;
            this.innerHeight = 0;
        }
        
        ElevatorStates() {
            //refrence HOME instead somehow
            this(0.57, 0.01, 210);
        }
    }

    /*
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
    */


    public enum IntakeStates {
        // allow for easy changing of elevator states 
        STOP(0),
        INTAKE(1),
        FEED_OUT(-1);

        public final double speed;

        IntakeStates(double speed) {
            this.speed = speed;
        }
    }


    public enum ClimbStates {
        STOP(0),
        CLIMB(1),
        DROP(-1);

        public final double speed;

        ClimbStates(double speed) {
            this.speed = speed;
        }
    }
}
