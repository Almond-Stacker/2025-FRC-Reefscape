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
            this(0.57, 0.01, 210);
        }
    }

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
