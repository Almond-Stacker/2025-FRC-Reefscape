package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.PhotonSubsystem;

public class PhotonCommand {
    
    private PhotonSubsystem photonSubsystem;

    public PhotonCommand(PhotonSubsystem photonSubsystem) {
        this.photonSubsystem = photonSubsystem;
    }

    public Command goToPose(Pose2d targetPose) {

    }

    public PhotonSubsystem getPhotonSubsystem() {
        return photonSubsystem;
    }

}
