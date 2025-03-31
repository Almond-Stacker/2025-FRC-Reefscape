package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.PhotonConsts;

public class States {
    public enum ElevatorStates {
        MIN(0, 3.211, 60),
        MAX(113 , 29,217),
        STARTING_POSITION(18, 23,74),
        PRE_INTAKE(73.55, 26, 67),
        HIGH_PRE_INTAKE(73.55, 28, 98),
        INTAKE(73.55, 15.6, 67),
        L1(38,23.66,116),
        L2(15, 20.13, 214), //(1,7.416,203),
        L3(28.2011, 19.13, 214), // angle 200
        L4(108, 25.5, 215),

        // NEVER SET THE ELEVATORS TO THIS STATE
        // ONLY SET THE ARM 
        CLIMB(0,0, 180);
        public final double primaryHeight; 
        public final double innerHeight;
        public final double slow;

        public final double armAngle;

        ElevatorStates(double primaryHeight, double innerHeight, double intakeArmAngle, double slow) {
            this.primaryHeight = primaryHeight;
            this.innerHeight = innerHeight;
            this.armAngle = intakeArmAngle;
            this.slow = slow;
        }

        ElevatorStates(double primaryHeight, double innerHeight, double intakeArmAngle) {
            this.primaryHeight = primaryHeight;
            this.innerHeight = innerHeight;
            this.armAngle = intakeArmAngle;
            this.slow = 1;
        }
    }

    public enum IndexStates {
        INTAKE(1),
        OUTTAKE(-1),
        STOP(0.02);

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

    public enum ReefPosition {
        // was 0.17 
        // LEFT(-0.13),//0.185
        // CENTER(0.0),//was 20 for left reef - left cam, right reef - right cam
        // RIGHT(0.17);
        LEFT(-0.15),//0.185 // was 0.18 durring practive - Vegas
        CENTER(0.0),//was 20 for left reef - left cam, right reef - right cam
        RIGHT(0.225);//was 0.225 durring practice match - Vegas
        public final double yOffset;
        public final Transform2d tagToRobot;
        
        ReefPosition(double yOffset) {
            this.yOffset = yOffset;
            this.tagToRobot = 
                new Transform2d(
                    PhotonConsts.CENTER_TO_TAG_DELTA_X_DEFAULT,
                    yOffset,
                    Rotation2d.kPi
                );//not used currently 3/27/25
        }
    }
}
