package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonSubsystem;

public class positionRelativeToAprilTag extends Command{
    private PhotonSubsystem camera;

    private double desiredDistance;
    private double desiredAngle;
    private double targetHeightOffGround;
    private int targetID; 

    public positionRelativeToAprilTag(PhotonSubsystem camera, int targetID, double desiredDistance, double desiredAngle, double targetHeightOffGround) {
        this.camera = camera;
        this.desiredAngle = desiredAngle;
        this.desiredDistance = desiredDistance;
        this.targetHeightOffGround = targetHeightOffGround;
        this.targetID = targetID;
    }

    @Override
    public void initialize() { 
        camera.setTargetID(targetID);
        camera.setDesiredAngle(desiredAngle);
        camera.setDesiredDistance(desiredDistance);
        camera.setTargetHeightOffGround(targetHeightOffGround);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
