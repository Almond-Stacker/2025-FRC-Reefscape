package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConsts;
import frc.robot.commands.PhotonCommand;

//built for multiple cameras, finds average pose of every reading then 
public class PhotonSubsystem extends SubsystemBase {
    private PhotonCommand commands;

    private Matrix<N3, N1> curStdDevs = PhotonConsts.SINGLE_STD_DEVS; // dynamic standard deviation

    private List<PhotonCamera> cameras; // pain transforms and cameras later
    private List<Transform3d> cameraToRobotTransforms;
    private final PhotonPoseEstimator estimator;
    private Optional<EstimatedRobotPose> collectiveEstimatedPose = Optional.empty();

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    private double lastUpdateTime = 0;
    
        public PhotonSubsystem(List<String> cameraNames, List<Transform3d> cameraToRobotTransforms) {
            this.cameraToRobotTransforms = cameraToRobotTransforms;//was throwin error
            this.cameras = cameraNames.stream().map(PhotonCamera::new).toList();
            this.estimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, //figure custom April Tag feld creation
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraToRobotTransforms.get(0) // Using first cameras transform as refrence
            );
        }
    
        //for swerve drive estimator type stuff
        private void updateEstimationStdDevs(int numTags, double avgDist) {
                //if 1 and far then standard deviation is larger, else smaller
            curStdDevs = numTags > 1 ? VecBuilder.fill(0.5, 0.5, 1) : VecBuilder.fill(4, 4, 8);
            if(numTags == 1 && avgDist > 4) {//dist in meters
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
            Pose3d addedPose = new Pose3d(); // find average pose, couldn't figure a List or array list for some reason
            double avgDist = 0;//for standard deviation
            int validPoseCount = 0; //for average pose among cameras, could also just get best target overall and best camera
            int index = 0;//which cam for pose transforms
    
            double currentTime = Timer.getFPGATimestamp();
    
            for(PhotonCamera cam : cameras) {
                List<PhotonPipelineResult> results = cam.getAllUnreadResults();
                for(PhotonPipelineResult result : results) {
    
                    //check if targets are resonable
                    List<PhotonTrackedTarget> filteredTargets = result.getTargets().stream()
                        .filter(target -> target.getPoseAmbiguity() < PhotonConsts.MIN_AMBIGUITY)
                        .toList();
    
                    if(!filteredTargets.isEmpty()) {
                        Optional<EstimatedRobotPose> pose = estimator.update(result);
                        //check if pose was resonable
                        if(pose.isPresent()) {
                            //transforms here to utalize index, consider creating a higher level pose handler
                            Pose3d transformedPose = pose.get().estimatedPose.transformBy(cameraToRobotTransforms.get(index).inverse());
                            avgDist += transformedPose.toPose2d().getTranslation().getNorm();
                            validPoseCount++;
                        }
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
                updateEstimationStdDevs(validPoseCount, avgDist / validPoseCount);
                lastUpdateTime = currentTime;
            collectiveEstimatedPose = Optional.of(new EstimatedRobotPose(averagedPose, 0, List.of(), PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
        } else if (currentTime - lastUpdateTime > PhotonConsts.TIMEOUT) {
            collectiveEstimatedPose = Optional.of(PhotonConsts.NO_APRILTAG_ESTIMATE); //clear result if too great of time difference
            //no change to swerve pose
        }

    }

    public Optional<EstimatedRobotPose> getCollectiveEstimatedPose() {
        return collectiveEstimatedPose;
    }

    //distance and angle to specific tag
    public void setSmartDashboard() {
        
    }

    public PhotonCommand getCommands() {
        return commands;
    }
}
