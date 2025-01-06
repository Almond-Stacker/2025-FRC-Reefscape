
package frc.robot.subsystems;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PhotonConfig;
import frc.robot.generated.TunerConstants;

public class PhotonVision extends SubsystemBase{
    PhotonCamera camera;
    PhotonPipelineResult lastestDetection;
    PhotonTrackedTarget target;

    Transform3d targetLocation; 
    PhotonConfig configuration;

    double targetRange;
    double targetYaw;
   public  boolean targetSeen;
    double hd;
    double ad;

    //test commit big balls

    public PhotonVision(PhotonConfig configuration) {
        this.configuration = configuration;
        camera = new PhotonCamera(configuration.cameraName);
    }

    @Override 
    public void periodic(){
        lastestDetection = camera.getLatestResult();
        if(lastestDetection != null) {
            targetSeen = false;
            // look through all targets and check their target detection
            for(PhotonTrackedTarget x: lastestDetection.getTargets()) {
                if(x.getFiducialId() == configuration.targetID) {
                    targetSeen = true;
                    targetYaw = x.getYaw();
                    // targetRange = PhotonUtils.calculateDistanceToTargetMeters( 
                    //     configuration.cameraHeightOffGround, 
                    //     configuration.targetHeightOffGround, 
                    //     Units.degreesToRadians(configuration.cameraPitch), 
                    //     Units.degreesToRadians(x.getPitch()));
                        
                        hd = configuration.cameraHeightOffGround - configuration.targetHeightOffGround;
                        ad = Units.degreesToRadians(x.getPitch());
                        SmartDashboard.putNumber("Pitch", ad);
                        targetRange = hd / (Math.tan(ad));

                        targetLocation = x.getBestCameraToTarget();
                        SmartDashboard.putNumber("trange", targetRange);
                        SmartDashboard.putNumber("tyaw", targetYaw);
                        break;
                }

            }
        }
        else {
            targetSeen = false;
        }

        SmartDashboard.putBoolean("targetSeen", targetSeen);
        
    }

    public double getYaw() {
        return targetYaw;
    }

    public double getDistance() {
        return targetRange;
    }
    
    public Transform3d getPosition() {
        return targetLocation;
    }

    public double getForwardOutput() {
        return -(configuration.targetDistance - getDistance()) * TunerConstants.kSpeedAt12VoltsMps;
    }
    
    // public double getTurnOutput() {
    //     return -(configuration.targetYaw - getYaw()) * TunerConstants.kSpeedAt12VoltsMps * 0.06;
    // }

    public double getTurnOutput() {
        return -1 * getYaw() * 0.04 * TunerConstants.kSpeedAt12VoltsMps;
    }

}