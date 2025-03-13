package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final Supplier<Double> currentRobotYaw; 
    private final boolean allianceBlue;
    private final SwerveDrivePoseEstimator3d poseEstimator; 
    private final CommandSwerveDrivetrain drivetrain; 
    private SwerveModulePosition[] positions = new SwerveModulePosition[4]; 

    private EstimatedRobotPose estimatedRobotPose;
    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult pipelineResult;
    private Transform3d tagLocation;
    private Pose3d tagPose; 
    private Translation3d tagTranslation;
    private Rotation3d tagRotation; 
    private Timer timer;

    private double targetYaw;    
    private double targetPitch;
    private double targetRoll;
    private int targetID; 

    private boolean targetSeen;

    public PhotonSubsystem(String cameraName, Supplier<Double> currentRobotYaw, CommandSwerveDrivetrain driveTrain) {
        camera = new PhotonCamera(cameraName);
        this.currentRobotYaw = currentRobotYaw;
        this.drivetrain = driveTrain;
        for(int i = 0; i < 4; i++) {
            positions[i] = drivetrain.getModule(i).getPosition(true);
        }

        poseEstimator = new SwerveDrivePoseEstimator3d(drivetrain.getKinematics(), drivetrain.getRotation3d(), positions, new Pose3d());
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            allianceBlue = true;
        } else {
            allianceBlue = false;
        }
    }

    @Override 
    public void periodic() {
        results = camera.getAllUnreadResults();
        targetSeen = false;
        
        // get most recent result
        if(pipelineResult == null) {
            targetID = 10000;
            return;
        }
        pipelineResult = results.get(0);
        if(pipelineResult.hasTargets()) {
            for(PhotonTrackedTarget target: pipelineResult.getTargets()) {
                targetSeen = true;
                targetID = target.getFiducialId();
                tagTranslation = new Translation3d(tagLocation.getX(), tagLocation.getY(), tagLocation.getZ());
                tagRotation = new Rotation3d(0, target.getPitch(), target.getYaw());
                tagPose = new Pose3d(tagTranslation, tagRotation);
                tagLocation = target.getBestCameraToTarget();
                poseEstimator.addVisionMeasurement(tagPose, Timer.getFPGATimestamp());
            }
        }   
        tagTranslation = new Translation3d(tagLocation.getX(), tagLocation.getY(), tagLocation.getZ());
    
        estimatedRobotPose = new EstimatedRobotPose(tagPose, Timer.getFPGATimestamp(), pipelineResult.getTargets(), PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
        setSmartDashboardValues();
    }

    public Pose3d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Transform3d getTagLocation() {
        return tagLocation;
    }

    public double getYaw() {
        if(allianceBlue) {
            switch (targetID) {
                case 1:
                    return 30 - currentRobotYaw.get();
                case 2:
                    return 30 - currentRobotYaw.get();
                case 3:
                    return 30 - currentRobotYaw.get();
                case 4:
                    return 30 - currentRobotYaw.get();
                case 5:
                    return 30 - currentRobotYaw.get();
                case 6:
                    return 30 - currentRobotYaw.get();
                default:
                    return 0; 
            } 
        } else {
            switch (targetID) {
                case 1:
                    return 30 - currentRobotYaw.get();
                case 2:
                    return 30 - currentRobotYaw.get();
                case 3:
                    return 30 - currentRobotYaw.get();
                case 4:
                    return 30 - currentRobotYaw.get();
                case 5:
                    return 30 - currentRobotYaw.get();
                case 6:
                    return 30 - currentRobotYaw.get();
                default:
                    return 0; 
            }
        }
    }

    public int getTagID() {
        return targetID;
    }

    private void setSmartDashboardValues() {
        SmartDashboard.putNumber(camera.getName() + " target ID", targetID);
        SmartDashboard.putNumber(camera.getName() + " current Robot Yaw", currentRobotYaw.get());

        SmartDashboard.putBoolean(camera.getName() + " target seen", targetSeen);
    }
}
