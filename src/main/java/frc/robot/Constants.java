package frc.robot;

import edu.wpi.first.math.util.Units;

// put the algae down 
// this is for my processor 
public final class Constants {
    public static final class BeamBreak {
        public static final int beamBreakID = 0;
    }

    public static final class LED {
        public static final int LED_ID = 0;
    }

    public static final class Elevator {
        public static final int elevatorMotorID = 0; 
        public static final int encoderID = 0;
        public static final double velocitySetPoint = 0; 

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
    }

    // all distances must be in meters 
    // all angles must be in radians
    public static final class Photon {
        public static final double driveConstant = 1;
        public static final double angleConstant = 0.04;

        public static final class camera0 {
            public static final String cameraName = "front_photon_camera";
            public static final double cameraHeight = Units.inchesToMeters(23);
            public static final double cameraPitch = Units.degreesToRadians(0);
        }

        public static final class tag4 {
            public static final int targetID = 4;
            public static final double tagHeight = Units.inchesToMeters(21);
            public static final double distance0 = 0.3;
            public static final double angle0 = 4; 
        }
    }
}