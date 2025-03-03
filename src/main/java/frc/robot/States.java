package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class States {

    public enum ElevatorStates {
        //tune
        HOME_ABS(0.57, 0.1004, 210),
        MAX_ABS(101, 0.8, 255),
        INTAKE_ABS(0, 0.5, 80),
        MIN_ABS(0, -2.7, 70),
        HOME_REL(0.5, 210),
        L2_REL(.9, 210),
        L3_REL(1.5, 190),
        L4_REL(1.8, 180);
        

        public final double primaryHeight;
        public final double innerHeight;
        public final double relTotalHeight;
        public final double angle;
        public final boolean ABS;

        ElevatorStates(double primaryHeight, double innerHeight, double angle) {
            this.primaryHeight = primaryHeight;
            this.innerHeight = innerHeight;
            this.angle = angle;

            ABS = true;
            this.relTotalHeight = 0;//this.totalAbsToTotalRel(this.primaryHeight + this.innerHeight);
            SmartDashboard.putNumber("Relative Goal", relTotalHeight);

        }

        ElevatorStates(double relTotalHeight, double angle) {
            ABS = false;
            this.relTotalHeight = relTotalHeight;
            SmartDashboard.putNumber("Relative Goal", relTotalHeight);
            this.angle = angle;

            //don't need
            this.primaryHeight = 0;
            this.innerHeight = 0;
        }
        
        ElevatorStates() {
            this(0.57, 0.01, 210);
        }

        //private double totalAbsToTotalRel(double totalAbs) {
        //    return (totalAbs - .innerHeight)/(MAX_ABS.innerHeight - MIN_ABS.innerHeight)
        //        + (totalAbs - MIN_ABS.primaryHeight)/(MAX_ABS.primaryHeight - MIN_ABS.primaryHeight);
        //}
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
