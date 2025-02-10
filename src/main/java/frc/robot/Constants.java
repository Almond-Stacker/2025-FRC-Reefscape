package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
        public static final TrapezoidProfile.Constraints PROFILE = new TrapezoidProfile.Constraints(2, 4);
        
    }

    public static final class InnerElevatorConsts {
        public static final int elevatorMotorID = 22;

        public static final double kP = 0.38;
        public static final double kI = 0;
        public static final double kD = 0.03;

        public static final double kS = 0;
        public static final double kG = 0.03;
        public static final double kV = 0;

        public static final double gravityNegationConstant = 0.7;
        public static final TrapezoidProfile.Constraints PROFILE = new TrapezoidProfile.Constraints(2, 4);
    }

    public static final class IntakeArmConsts {
        public static final int armMotorID = 17;
        public static final int encoderID = 0;
        public static final int suckMotorID = 21;
        public static final double OUT_TIMEOUT = 1;//one second 

        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 1);
        public static final double kS = 0;
        public static final double kG = 0.003;
        public static final double kV = 0;
    }

    public static final class PhotonConsts {
        public static final String CAM0_NAME = "front_photon_camera";
        public static final String CAM1_NAME = "";//set constants 2/9

        public static final double TIMEOUT = 0.1;
        public static final double MIN_AMBIGUITY = 0.2;//tune

        //no reading consideration
        public static final Matrix<N3, N1> SINGLE_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
        public static final PhotonPipelineResult NO_RESULT = new PhotonPipelineResult();
        public static final Pose3d NO_APRILTAG = new Pose3d();
        public static final EstimatedRobotPose NO_APRILTAG_ESTIMATE =
                new EstimatedRobotPose(NO_APRILTAG, 0, List.of(), PoseStrategy.LOWEST_AMBIGUITY);

        
    }

}
