package frc.robot.command;
//https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner/src/main/java/frc/robot stupid sigma pay wall methods
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.generated.TunerConstants;

// exactly the same as the code from the photon vision website 
public class ram extends Command{
    DoubleSupplier ySupplier;
    DoubleSupplier xSupplier;
    DoubleSupplier rotationSupplier;
    PhotonVision camera; 
    SwerveRequest.RobotCentric swerveController = new SwerveRequest.RobotCentric(); 
    
    double desiredAngle;
    double desiredDistance; 
    double strafe; 
    double forward;
    double turn; 
    double safteyConstant = 0.1;

    CommandSwerveDrivetrain swerve;

    public ram(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, PhotonVision camera, CommandSwerveDrivetrain driveTrain) {
        this.xSupplier = x;
        this.ySupplier = y;
        this.rotationSupplier = rotation;
        this.camera = camera; 
        this.swerve = driveTrain;
        this.desiredAngle = 0;
        this.desiredDistance = 0.5;
    }

    @Override
    public void execute() {
        forward = -xSupplier.getAsDouble() * TunerConstants.kSpeedAt12VoltsMps;
        strafe = -ySupplier.getAsDouble() * TunerConstants.kSpeedAt12VoltsMps;
        turn = -rotationSupplier.getAsDouble() * TunerConstants.kSpeedAt12VoltsMps;

        turn = (desiredAngle - camera.getYaw()) * TunerConstants.kSpeedAt12VoltsMps * safteyConstant;
       forward = -(desiredDistance - camera.getDistance()) * TunerConstants.kSpeedAt12VoltsMps * safteyConstant;
       SmartDashboard.putNumber("turn", turn);
       SmartDashboard.putNumber("foward", forward);
       swerve.applyRequest(() -> swerveController
            .withVelocityX(forward)
            .withVelocityY(strafe)
            .withRotationalRate(turn));
}

    // @Override
    // public void end(boolean isFinished) {
    //     swerve.applyRequest(() -> swerveController
    //         .withVelocityX(0)
    //         .withVelocityY(0)
    //         .withRotationalRate(0));
    // }

    @Override
    public boolean isFinished() {
        if(Math.abs(desiredAngle - camera.getYaw()) <= 4 && Math.abs(desiredDistance - camera.getDistance()) <= 0.2) {
            return true;
        }
        return false;
    }
}
