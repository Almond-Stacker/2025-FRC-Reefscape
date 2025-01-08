package frc.robot;

import edu.wpi.first.math.util.Units;

// put the algae down 
// this is for my processor 
public final class Constants {

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
            public static final double tagHeight = Units.inchesToMeters(22);
            public static final double distance0 = 0.3;
            public static final double angle0 = 4; 
        }
    }
}