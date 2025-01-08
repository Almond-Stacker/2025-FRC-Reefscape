package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.PhotonStates;
import frc.robot.subsystems.PhotonSubsystem;

public class positionRelativeToAprilTag extends Command{
    private PhotonSubsystem camera;
    private PhotonStates state;

    public positionRelativeToAprilTag(PhotonSubsystem camera, PhotonStates state) {
        this.camera = camera;
        this.state = state;
    }

    @Override
    public void initialize() { 
        camera.setState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
