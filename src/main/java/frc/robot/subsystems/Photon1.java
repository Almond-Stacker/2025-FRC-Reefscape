package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.lang.reflect.Field;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon1  extends SubsystemBase{
    private final PhotonCamera camera; 
    private final Transform3d cameraToRobot; 
    private final CommandSwerveDrivetrain drive;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;
    

    public Photon1(CommandSwerveDrivetrain drive) {
        camera = new PhotonCamera("gray_photon_camera");
        cameraToRobot = new Transform3d();
        this.drive = drive;
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);
    }

    @Override
    public void periodic() {
        if(camera.getAllUnreadResults() == null || camera.getAllUnreadResults().isEmpty()) {
            return; 
        }

        PhotonPipelineResult tar = camera.getAllUnreadResults().get(0);
        if(!tar.hasTargets()) {
            return;
        }
        

        PhotonTrackedTarget sigma  = tar.getBestTarget();
       var sigma1 = PhotonUtils.estimateCameraToTarget(sigma.getBestCameraToTarget().getTranslation().toTranslation2d(), fieldLayout.getTagPose(sigma.getFiducialId()).get().toPose2d(), new Rotation2d(drive.getRotation3d().getAngle()));
    SmartDashboard.putNumber("sheoaum", sigma.yaw);
    }
}