package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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

    private Transform2d sigma;
    private double yaw1;
    double avgx;
    double avgy;

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
        setSmartDashboardValues();

        // get most recent result
        if(results == null) {
            targetID = 10000;
            return;
        }
        if(results.isEmpty() ) {
            targetID = 10000;
            return;
        }

        pipelineResult = results.get(0);
        if(pipelineResult.hasTargets()) {
            for(PhotonTrackedTarget target: pipelineResult.getTargets()) {
                targetSeen = true;
                System.out.println("sigma boy");
                targetID = target.getFiducialId();
               tagLocation = target.getBestCameraToTarget();
                tagTranslation = new Translation3d(tagLocation.getX(), tagLocation.getY(), tagLocation.getZ());
                tagRotation = new Rotation3d(0, Units.degreesToRadians(target.getPitch()), Units.degreesToRadians(target.getYaw()));
                SmartDashboard.putNumber(camera.getName() +"sigma rt", Units.degreesToRadians(target.getYaw()));

                tagPose = new Pose3d(tagTranslation, tagRotation);

                 avgx = 0; 
                 avgy = 0; 

                for(Translation2d x : drivetrain.getModuleLocations()) {
                    avgx += x.getX();
                    avgy += x.getY();
                }
                avgx /= 4;
                avgy /= 4;
                yaw1 = PhotonUtils.getYawToPose(new Pose2d(avgx, avgy, drivetrain.getRotation3d().toRotation2d()), 
                    new Pose2d(tagTranslation.getX(), tagTranslation.getY(), new Rotation2d(target.getYaw()))).getDegrees();
                sigma = PhotonUtils.estimateCameraToTarget(new Translation2d(tagTranslation.getX(), tagTranslation.getY()), 
                new Pose2d(1,1, new Rotation2d(0)), drivetrain.getRotation3d().toRotation2d());

                SmartDashboard.putNumber(camera.getName() + " ntheauo", yaw1);
                SmartDashboard.putNumber(camera.getName() + " sigm1 x", sigma.getX());
                SmartDashboard.putNumber(camera.getName() + " sigm1 y", sigma.getY());

                SmartDashboard.putNumber(camera.getName() + " sigm1 r", sigma.getRotation().getDegrees());


                poseEstimator.updateWithTime(Timer.getFPGATimestamp(), drivetrain.getPigeon2().getRotation3d(), positions);

                poseEstimator.addVisionMeasurement(tagPose, Timer.getFPGATimestamp());
                for(int i = 0; i < 4; i++) {
                    positions[i] = drivetrain.getModule(i).getPosition(true);
                }
                
                estimatedRobotPose = new EstimatedRobotPose(tagPose, Timer.getFPGATimestamp(), pipelineResult.getTargets(), PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
                setSmartDashboardValues();

                SmartDashboard.putNumber(camera.getName() + "sigma x", this.getRobotPose().getX());
                SmartDashboard.putNumber(camera.getName() +"sigma y", this.getRobotPose().getY());
        
                SmartDashboard.putNumber(camera.getName() +"sigma r", this.getRobotPose().getRotation().getAngle());
                SmartDashboard.putNumber(camera.getName() + "tag x", tagLocation.getX());
                SmartDashboard.putNumber(camera.getName() + "tag y", tagLocation.getY());
            }
        }   
    
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
