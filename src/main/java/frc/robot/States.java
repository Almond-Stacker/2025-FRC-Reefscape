package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.PhotonConsts;

public class States {
    public enum ElevatorStates {
        MIN(0, 3.211, 40),
        MAX(112 , 29,220),
        STARTING_POSITION(0.06, 23,71),
        PRE_INTAKE(77.55, 25.5, 90),
        INTAKE(77.55, 19.5, 57),
        L1(21,24,110),
        L2(21, 24, 140), //(1,7.416,203),
        L3(23.9, 24, 200), // angle 200
        L4(108.5, 25.5, 220);
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
        STOP(0.05);

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
        LEFT(-0.165),
        CENTER(0.0),
        RIGHT(0.165);

        public final double yOffset;
        public final Transform2d tagToRobot;
        
        ReefPosition(double yOffset) {
            this.yOffset = yOffset;
            this.tagToRobot = 
                new Transform2d(
                    PhotonConsts.CENTER_TO_TAG_DELTA_X,
                    yOffset,
                    Rotation2d.kPi
                );
        }
    }
}
