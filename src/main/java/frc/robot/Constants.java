package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

// put the algae down 
// this is for my processor 
public final class Constants {
    // public static final class Swerve {
    //     public static SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
    //         new Translation2d(TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos),
    //         new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos),
    //         new Translation2d(TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos),
    //         new Translation2d(TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos)
    //     );
    // }

    public static final class BeamBreak {
        public static final int beamBreakID = 0;
    }

    public static final class LED {
        public static final int LED_ID = 0;
    }

    public static final class AlgaeIntake {
        public static final int intakeMotorID = 20; 
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    // FINISHED 
    public static final class PrimaryElevator {

        public static final int leftElevatorMotorID = 14;
        public static final int rightElevatorMotorID = 15;
        public static final int encoderID = 1;
        public static final double velocitySetPoint = 0; 

        public static final double kP = 0.02;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
    }

    // final 
    public static final class InnerElevator {
        public static final int ElevatorMotorID = 22;

        public static final double kP = 0.38;
        public static final double kI = 0;
        public static final double kD = 0.03;
        public static final double kS = 0;
        public static final double kG = 0.03;
        public static final double kV = 0;
    }

    // FINISHED 
    public static final class Arm {
        public static final int armMotorID = 17;
        public static final int encoderID = 0;
        public static final int indexingMotorID = 21; 

        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0.003;
        public static final double kV = 0;
    }

    // all distances must be in meters 
    // all angles must be in radians
    public static final class Photon {
        public static final double driveConstant = 1;
        public static final double angleConstant = 0.06;

        public static final class camera0 {
            public static final String cameraName = "front_photon_camera";
            public static final double cameraHeight = Units.inchesToMeters(11);
            public static final double cameraPitch = Units.degreesToRadians(0);
        }

        public static final class tag4 {
            public static final int targetID = 4;
            public static final double tagHeight = Units.inchesToMeters(21);
            public static final double distance0 = 0.3;
            public static final double angle0 = 4; 
        }

        public static final class tag1 {
            public static final int targetID = 1;
            public static final double tagHeight = Units.inchesToMeters(8.2);
            public static final double desiredDistance1 = 0.5;
            public static final double desiredAngle1 = 0;
    
        }
    }
}