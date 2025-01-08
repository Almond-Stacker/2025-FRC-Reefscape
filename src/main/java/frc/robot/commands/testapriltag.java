package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.PhotonStates;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonSubsystem;

public class testapriltag extends Command{
    private CommandSwerveDrivetrain swerve;
    private SwerveRequest.RobotCentric robotCentricDrive;
    private PhotonSubsystem camera;
    private PhotonStates state;
    private DoubleSupplier strafe;

    public testapriltag(CommandSwerveDrivetrain swerve, PhotonSubsystem camera, PhotonStates state, DoubleSupplier strafe) {
        this.swerve = swerve;
        this.camera = camera;
        this.state = state;
        this.strafe = strafe;
    }

    @Override
    public void initialize() {
        camera.setState(state);
    }

    @Override 
    public void execute() {
        swerve.applyRequest(() -> robotCentricDrive.withVelocityX(camera.getForwardOutput())
                .withVelocityY(strafe.getAsDouble())
                .withRotationalRate(camera.getTurnOutput()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.applyRequest(() -> robotCentricDrive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override 
    public boolean isFinished() {
        if(camera.getRange() <= state.distance && camera.getYaw() <= state.angle) {
            return true;
        }
        return false;
    }
}
