package frc.robot;

public class PhotonConfig {
    public final String cameraName;
    public final int targetID;
    public final double cameraHeightOffGround;
    public final double targetHeightOffGround;
    public final double cameraPitch;
    public final double targetYaw;
    public final double targetDistance;

    public PhotonConfig(String cameraName, int taregtID, double cameraHeightOffGround, double targetHeightOffGround, double cameraPitch, double targetYaw, double targetDistance) {
        this.cameraName = cameraName;
        this.targetID = taregtID;
        this.cameraHeightOffGround = cameraHeightOffGround;
        this.targetHeightOffGround = targetHeightOffGround;
        this.cameraPitch = cameraPitch;
        this.targetYaw = targetYaw;
        this.targetDistance = targetDistance;
    }
}