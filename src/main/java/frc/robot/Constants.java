package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.Arrays;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

//if want to import with .* to avoid typing exact class dependances
//consider seperate constants files
public class Constants {
    
    public static final class PrimaryElevatorConsts {
        public static final int leftElevatorMotorID = 14;
        public static final int rightElevatorMotorID = 15;
        public static final int encoderID = 1;

        //temopory estamate, although resonable value?
        //smaller the better, however don't want occelations 
        public static final double PID_TOLERANCE = 10;
        public static final double kP = 0.02;
        public static final double kI = 0;
        public static final double kD = 0;
        
        //tune this as well too, could be very off ##note it's reading height
        public static final TrapezoidProfile.Constraints PROFILE = new TrapezoidProfile.Constraints(Double.MAX_VALUE, Double.MAX_VALUE);
        
    }

    public static final class InnerElevatorConsts {
        public static final int elevatorMotorID = 22;

        public static final double kP = 0.36;
        public static final double kI = 0;
        public static final double kD = 0.03;

        public static final double kS = 0;
        public static final double kG = 0.08;
        public static final double kV = 0;

        public static final double gravityNegationConstant = 0.7;
        public static final TrapezoidProfile.Constraints PROFILE = new TrapezoidProfile.Constraints(Double.MAX_VALUE, Double.MAX_VALUE);
    }

    public static final class IntakeArmConsts {
        public static final int armMotorID = 17;
        public static final int encoderID = 0;
        public static final int suckMotorID = 21;
        public static final double OUT_TIMEOUT = 1;//one second 

        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final TrapezoidProfile.Constraints PROFILE = new TrapezoidProfile.Constraints(0.1, 0.03);
        public static final double kS = 0;
        public static final double kG = 0.008;
        public static final double kV = 0;
    }

    public static final class PhotonConsts {
        public static final List<String> CAM_NAMES = Arrays.asList("gray_photon_camera", "blue_photon_camera");

        public static final Transform3d blue_cam_transform = new Transform3d(
                    new Translation3d(-0.1956816, 0.2815336, 0),
                    new Rotation3d(0, 0, -0.43633231)
                );

        public static final Transform3d gray_cam_transform = new Transform3d(
                    new Translation3d(-0.1956816, -0.2815336, 0),
                    new Rotation3d(0, 0, 0.43633231)
                );

        //MEANSURE TRANSFORMATIONS CAM --> ROBOT 2/27
        public static final List<Transform3d> CAM_TO_ROBOT_TRANSFORMS = List.of(gray_cam_transform, blue_cam_transform);
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

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

    public static final class ClimbConsts {
        public static final int leftClimbMotorID = 21; //find
        public static final int rightClimbMotorID = 20;
    }

}
