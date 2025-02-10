package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConsts;
import frc.robot.commands.PhotonCommand;

//built for multiple cameras, finds average pose of every reading then 
public class PhotonSubsystem extends SubsystemBase {
    private PhotonCommand commands;

    private Matrix<N3, N1> curStdDevs = VecBuilder.fill(4, 4, 8); // dynamic standard deviation

    private List<PhotonCamera> cameras; // pain transforms and cameras later
    private List<Transform3d> cameraToRobotTransforms;
    private final PhotonPoseEstimator estimator;
    private Optional<EstimatedRobotPose> collectiveEstimatedPose = Optional.empty();

    public PhotonSubsystem(List<String> cameraNames, List<Transform3d> cameraToRobotTransforms) {
        this.cameraToRobotTransforms = cameraToRobotTransforms;//was throwin error
        this.cameras = cameraNames.stream().map(PhotonCamera::new).toList();
        this.estimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), //figure custom April Tag feld creation
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraToRobotTransforms.get(0) // Using first cameras transform as refrence
        );
    }

    /* 

    private void updateEstimationStdDevs(List<EstimatedRobotPose> poses) {
        if(poses.isEmpty()) {
            curStdDevs = VecBuilder.fill(4, 4, 8);
            return;
        }
        int numTags = poses.size();
        double avgDist = 0;
        for(EstimatedRobotPose pose : poses) {
            avgDist += pose.estimatedPose.getTranslation().getNorm();
        }
        avgDist /= numTags;

        if(numTags > 1) {
            curStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        }
        //if far then screw that, if good enough, take average distance squared
        if(numTags == 1 && avgDist > 4) {
            curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            curStdDevs = curStdDevs.times(1 + (avgDist * avgDist / 30));
        }
    }
    */

    //for swerve drive estimator type stuff
    private void updateEstimationStdDevs(int numTags) {
        double avgDist = 0;
        //if 1 and far then standard deviation is larger, else smaller
        curStdDevs = numTags > 1 ? VecBuilder.fill(0.5, 0.5, 1) : VecBuilder.fill(4, 4, 8);
        if(numTags == 1 && avgDist > 4) {
            curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            curStdDevs = curStdDevs.times(1 + (avgDist * avgDist / 30));
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @Override
    public void periodic() {
        Pose3d addedPose = new Pose3d(); // find average pose
        int validPoseCount = 0; //for average pose among cameras, could also just get best target overall and best camera
        int index = 0;//cam for pose transforms

        for(PhotonCamera cam : cameras) {
            List<PhotonPipelineResult> results = cam.getAllUnreadResults();
            for(PhotonPipelineResult result : results) {
                Optional<EstimatedRobotPose> pose = estimator.update(result);
                if(pose.isPresent()) {
                    Pose3d transformedPose = pose.get().estimatedPose.transformBy(cameraToRobotTransforms.get(index).inverse());
                    addedPose = addedPose.plus(new Transform3d(transformedPose.getTranslation(), transformedPose.getRotation()));
                    validPoseCount++;
                }
            }
            index++;
        }

        if(validPoseCount > 0) {
            Pose3d averagedPose = addedPose.transformBy(new Transform3d(
                addedPose.getX() / validPoseCount,
                addedPose.getY() / validPoseCount,
                addedPose.getZ() / validPoseCount,
                new Rotation3d(
                    addedPose.getRotation().getX() / validPoseCount,
                    addedPose.getRotation().getY() / validPoseCount,
                    addedPose.getRotation().getZ() / validPoseCount
                )
            ));
            updateEstimationStdDevs(validPoseCount);
            collectiveEstimatedPose = Optional.of(new EstimatedRobotPose(averagedPose, 0, List.of(), PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
        }

    }

    public Optional<EstimatedRobotPose> getCollectiveEstimatedPose() {
        return collectiveEstimatedPose;
    }

    public PhotonCommand getCommands() {
        return commands;
    }
}
