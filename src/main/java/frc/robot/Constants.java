package frc.robot;

import java.util.Arrays;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

        public static final double kP = 0.02;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final TrapezoidProfile.Constraints TRAPEZOID_PROFILE =
            new TrapezoidProfile.Constraints(0,0);

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

        public static final double gravityNegationConstant = 0.7;
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
    // public static final class Photon {
    //     public static final double driveConstant = 0.5;//;
    //     public static final double angleConstant = 0.06;

    //     public static final class camera0 {
    //         public static final String cameraName = "left_photon_camera";
    //         public static final double cameraHeight = Units.inchesToMeters(11);
    //         public static final double cameraPitch = Units.degreesToRadians(0);
    //     }

    //     public static final class tag4 {
    //         public static final int targetID = 4;
    //         public static final double tagHeight = Units.inchesToMeters(21);
    //         public static final double distance0 = 0.3;
    //         public static final double angle0 = 4; 
    //     }

    //     public static final class tag1 {
    //         public static final int targetID = 1;
    //         public static final double tagHeight = Units.inchesToMeters(8.2);
    //         public static final double desiredDistance1 = 0.5;
    //         public static final double desiredAngle1 = 0 ;
    
    //     }
    // }

        public static final class PhotonConstants {
            public static final List<String> CAM_NAMES = Arrays.asList("front_photon_camera");

            public static final Transform3d front_cam_transform = new Transform3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0, 0, 0)
                    );
            public static final List<Transform3d> CAM_TO_ROBOT_TRANSFORMS = List.of(front_cam_transform);
           // public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

            // public static final String CAM0_NAME = "front_photon_camera";
            //public static final String CAM1_NAME = "";//set constants 2/9

            public static final double TIMEOUT = 0.3;
            public static final double MIN_AMBIGUITY = 0.2;//tune

            //no reading consideration
            public static final Matrix<N3, N1> SINGLE_STD_DEVS = VecBuilder.fill(4, 4, 8);
            public static final Matrix<N3, N1> MULTI_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
            public static final PhotonPipelineResult NO_RESULT = new PhotonPipelineResult();
            public static final Pose3d NO_APRILTAG = new Pose3d();
            public static final EstimatedRobotPose NO_APRILTAG_ESTIMATE =
                    new EstimatedRobotPose(NO_APRILTAG, 0, List.of(), PoseStrategy.LOWEST_AMBIGUITY);

            public static final TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(2.0, 1.0);
            public static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2);
    }

    public static final class ClimbConstants {
        public static final int climbMotorID = 16; //find
    }
}