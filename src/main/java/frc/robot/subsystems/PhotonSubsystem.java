package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class PhotonSubsystem extends SubsystemBase{
    private PhotonCamera camera;
    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult pipelineResult;

    private double targetRange;
    private double targetYaw;
    private boolean targetSeen;
    private int targetID;

    private double cameraHeightOffGround;
    private double targetHeightOffGround;
    private double cameraPitch;
    private double desiredDistance;
    private double desiredAngle;


    public PhotonSubsystem(String cameraName, int targetID, double cameraHeightOffGround, double targetHeightOffGround, double cameraPitch, double desiredDistance, double desiredAngle) {
        camera = new PhotonCamera(cameraName);
        this.targetID = targetID;
        this.cameraHeightOffGround = cameraHeightOffGround;
        this.targetHeightOffGround = targetHeightOffGround;
        this.cameraPitch = Units.degreesToRadians(cameraPitch);
        this.desiredDistance = desiredDistance;
        this.desiredAngle = desiredAngle;
    }

    @Override 
    public void periodic(){
        results = camera.getAllUnreadResults();
        targetSeen = false;
        if(results.isEmpty()) {
            return;
        } 
        pipelineResult = results.get(results.size() - 1);
        if(pipelineResult.hasTargets()) {
            for(PhotonTrackedTarget target: pipelineResult.getTargets()) {
                if(target.getFiducialId() != targetID)
                {
                    break;
                }
                targetSeen = true;
                targetYaw = target.getYaw();
                targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                                cameraHeightOffGround, 
                                targetHeightOffGround, 
                                cameraPitch, 
                                Units.degreesToRadians(target.getPitch()));
            }
        }
    }   
    
    public void setTargetID(int id) {
        targetID = id;
    }

    public void setTargetHeightOffGround(double height) {
        targetHeightOffGround = height;
    }

    public void setCameraHeightOffGround(double height) {
        cameraHeightOffGround = height;
    }

    public double getForwardOutput() {
        if(targetSeen) {
            return -(desiredDistance - targetRange) * TunerConstants.kSpeedAt12Volts.magnitude();
        }
        return 0;
    }

    public double getTurnOutput() {
        if(targetSeen) {
           return (desiredAngle - targetYaw) * 0.04 * TunerConstants.kSpeedAt12Volts.magnitude();
        }
        return 0;
    }
}