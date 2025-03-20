package frc.robot.subsystems.VisionVersions;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.PhotonConsts;
import frc.robot.States.ReefPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision3 extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain; 

    private final PhotonCamera camera; 
    private final Transform3d cameraToRobot;
    private final PhotonPoseEstimator poseEstimator; 

    private List<PhotonPipelineResult> results;
    private ArrayList<PhotonTrackedTarget> currentTarget;
    private Transform3d tagToCamera; 
    private Transform2d tagToCameraProcessed; 
    private Pose2d estimatedRobotPoseRelative; 
    private EstimatedRobotPose estimatedRobotPoseField; 

    private int closestTarget; 
    private double leastDistance; 
    private boolean targetSeen; 
    
    public Vision3(String name, CommandSwerveDrivetrain drive, Transform3d cameraToRobot) {
        this.camera = new PhotonCamera(name);
        this.cameraToRobot = cameraToRobot;
        this.drivetrain = drive;
        poseEstimator = new PhotonPoseEstimator(Constants.PhotonConsts.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);
        this.targetSeen = false; 
    }

    @Override 
    public void periodic() {
        targetSeen = false; 
        results = camera.getAllUnreadResults();
        currentTarget = new ArrayList<>(0);

        Optional<PhotonPipelineResult> currentResult = Optional.empty();

        for(var change : results) {
            currentResult = Optional.of(change);
        }

        if(currentResult.isEmpty()) {
            SmartDashboard.putBoolean(camera.getName() + " target seen", targetSeen);
            return; 
        }
        
        closestTarget = -1; 
        leastDistance = Integer.MAX_VALUE;


        for(PhotonTrackedTarget target : currentResult.get().getTargets()) {
            targetSeen = true; 
            SmartDashboard.putBoolean(camera.getName() + " target seen", targetSeen);

            currentTarget.add(target);
            tagToCamera = target.getBestCameraToTarget();
            Pose2d tagFieldRelativePose = Constants.PhotonConsts.FIELD_LAYOUT.getTagPose(target.getFiducialId()).get().toPose2d();

            // MUST CHANGE GYRO IN ORDER TO FIT REQURIEMENTS
            // MUST BE 0 WHEN FACING OPPOSING ALLIANCE
            // MUST INCREASE WHEN MOVING COUNTER CLOCKWISE

            // calculate tag to camera transformation 
            // could possible end here for robot relative drive controls 
            tagToCameraProcessed = PhotonUtils.estimateCameraToTarget(tagToCamera.getTranslation().toTranslation2d(),
                    tagFieldRelativePose, 
                    drivetrain.getRotation3d().toRotation2d());  

            
            if(leastDistance > tagToCameraProcessed.getTranslation().getNorm()) {
                closestTarget = target.getFiducialId();
                leastDistance = tagToCameraProcessed.getTranslation().getNorm();
            }
            
            // alternative robot pose estimation could possibly need to be changed later 
            // current version could work out but not too sure 
            // estimatedRobotPoseRelative = PhotonUtils.estimateFieldToRobot(new Transform2d(tagToCamera.getTranslation().toTranslation2d(), tagToCamera.getRotation().toRotation2d()), 
            //     Constants.Photon.FIELD_LAYOUT.getTagPose(target.getFiducialId()).get().toPose2d(),
            //     new Transform2d(cameraToRobot.getTranslation().toTranslation2d(), cameraToRobot.getRotation().toRotation2d()));

            // calculate field relative pose for the robot 
            estimatedRobotPoseRelative = PhotonUtils.estimateFieldToRobot(tagToCameraProcessed, 
                tagFieldRelativePose,
                new Transform2d(cameraToRobot.getTranslation().toTranslation2d(), cameraToRobot.getRotation().toRotation2d()));


            // add vision measurment to drivetrain 
            estimatedRobotPoseField = poseEstimator.update(currentResult.get()).get();
            drivetrain.addVisionMeasurement(estimatedRobotPoseRelative, Timer.getFPGATimestamp());
            setSmartDashboardValues();
        }
        
    }

    public EstimatedRobotPose getFieldRelativePose() {
        return estimatedRobotPoseField; 
    }
    
    public Pose2d getRobotRelativePose() {
        return estimatedRobotPoseRelative;
    }

    public Pose2d getAprilTagPose2d(int tagId) {
        return PhotonConsts.FIELD_LAYOUT.getTagPose(tagId).get().toPose2d();
    }

    public Transform3d getBestTarget() {
        return tagToCamera; 
    }

    public boolean getTargetSeen() {
        return targetSeen; 
    }

    public int getClosestTarget() {
        return closestTarget; 
    }

    private void setSmartDashboardValues() {
        // camera to target readings 
        SmartDashboard.putNumber(camera.getName() + "estimated camera to target yaw", tagToCamera.getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber(camera.getName() + "estimated camera to target x", tagToCamera.getX());
        SmartDashboard.putNumber(camera.getName() + "estimated camera to target y", tagToCamera.getY());

        // sole photon vision estimated robot position 
        SmartDashboard.putNumber(camera.getName() + "estimated field relative robot yaw", getFieldRelativePose().estimatedPose.getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber(camera.getName() + "estimated field relative robot x", getFieldRelativePose().estimatedPose.getX());
        SmartDashboard.putNumber(camera.getName() + "estimated field relative robot y", getFieldRelativePose().estimatedPose.getY());

        SmartDashboard.putNumber(camera.getName() + "pidgeon readings ", drivetrain.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber(camera.getName() + " odometry readings yaw", drivetrain.getState().Pose.getRotation().getDegrees());

        // robot to target readings 
        SmartDashboard.putNumber(camera.getName() + "estimated robot to target yaw", estimatedRobotPoseRelative.getRotation().getDegrees());
        SmartDashboard.putNumber(camera.getName() + "estimated robot to target x", estimatedRobotPoseRelative.getX());
        SmartDashboard.putNumber(camera.getName() + "estimated robot to target y", estimatedRobotPoseRelative.getY());

    }

}