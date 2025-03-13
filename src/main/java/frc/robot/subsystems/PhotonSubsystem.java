package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final Supplier<Double> currentRobotYaw; 
    private final boolean allianceBlue;

    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult pipelineResult;
    private Transform3d tagLocation;

    private double targetYaw;    
    private int targetID; 

    private boolean targetSeen;

    public PhotonSubsystem(String cameraName, Supplier<Double> currentRobotYaw) {
        camera = new PhotonCamera(cameraName);
        this.currentRobotYaw = currentRobotYaw;

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
                tagLocation = target.getBestCameraToTarget();
            }
        }
        setSmartDashboardValues();
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
