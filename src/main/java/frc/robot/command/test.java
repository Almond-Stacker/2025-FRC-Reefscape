package frc.robot.command;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.generated.TunerConstants;

// exactly the same as the code from the photon vision website 
public class test extends Command{
    DoubleSupplier ySupplier;
    DoubleSupplier xSupplier;
    DoubleSupplier rotationSupplier;
    PhotonVision camera; 
    SwerveRequest.RobotCentric swerveController = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    double desiredAngle;
    double desiredDistance; 
    double strafe; 
    double forward;
    double turn; 
    PIDController pforward;
    PIDController pturn;

    CommandSwerveDrivetrain swerve;

    public test(CommandSwerveDrivetrain driveTrain) {
        this.swerve = driveTrain;
    }

    @Override
    public void execute() {
        swerve.applyRequest(() -> swerveController
            .withVelocityX(1)
            .withVelocityY(1)
            .withRotationalRate(1));
    }

    // @Override
    // public void end(boolean isFinished) {
    //     swerve.applyRequest(() -> swerveController
    //         .withVelocityX(0)
    //         .withVelocityY(0)
    //         .withRotationalRate(0));
    // }

    // @Override
    // public boolean isFinished() {
    //     if(Math.abs(desiredAngle - camera.getYaw()) <= 4 && Math.abs(desiredDistance - camera.getDistance()) <= 0.2) {
    //         return true;
    //     }
    //     return false;
    // }
}
