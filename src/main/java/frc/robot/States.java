package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Utilities;
import frc.robot.States.ElevatorStates;

public class States {

    public enum ElevatorStates {
        MIN(0, 1.2, 65),
        MAX(108 , 22.5,220),
        STARTING_POSITION(0, 19.3,71),
        HOME(0, 19.3, 71 ),
        L1(0,2.15,210),
        L2(0,1.25,210),
        L3(0, 20, 217),
        L4(105, 23, 195);
        /* 
        MIN(1, 1.2, 65),
        MAX(107 , 24,220),
        STARTING_POSITION(0, 19.3,71),
        HOME(0, 19.3, 71 ),
        HOME_REL(0.5, 71),
        PRE_INTAKE(0,21, 71),
        INTAKE(0,18.5,71),
        L1(0,0,0),
        L2(0,1.25,210),
        L3(0, 20, 217),
        L4(105, 23, 195);
        */
        public final double primaryHeight;
        public final double innerHeight;
        public final double totalRelativeHeight;
        public final double armAngle;

        ElevatorStates(double primaryHeight, double innerHeight, double intakeArmAngle) {
            this.primaryHeight = primaryHeight;
            this.innerHeight = innerHeight;
            this.armAngle = intakeArmAngle;
            this.totalRelativeHeight = Utilities.calculateTotalRelHeight(primaryHeight, innerHeight);
            //this.totalRelativeHeight = calculateTotalRelHeight(primaryHeight, innerHeight);
        }

        // ElevatorStates(double totalRelativeHeight, double armAngle) {
        //     this.primaryHeight = 0;
        //     this.innerHeight = 0;
        //     this.totalRelativeHeight = totalRelativeHeight;
        //     this.armAngle = armAngle;
        //     this.isABS = true;
        // }

        ElevatorStates() {
            this.primaryHeight = 0;
            this.innerHeight = 0;
            this.totalRelativeHeight = 0;
            this.armAngle = 0;
           // this.isABS = false;
        }

        //ik its not dynamic but idc, get outta here
        // private double calculateTotalRelHeight(double primaryHeight, double innerHeight) {
        //     double relPrimaryHeight = (primaryHeight - 1) / (107);//108 - 1
        //     double relInnerHeight = (innerHeight - 1.2) / (22.5 - 1.2);

        //     return relPrimaryHeight + relInnerHeight;
        // }
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
