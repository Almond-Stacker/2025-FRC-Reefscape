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
public class ramv1 extends Command{
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

    public ramv1(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, PhotonVision camera, CommandSwerveDrivetrain driveTrain) {
        this.xSupplier = x;
        this.ySupplier = y;
        this.rotationSupplier = rotation;
        this.camera = camera; 
        this.swerve = driveTrain;
        this.desiredAngle = 0;
        this.desiredDistance = 0.5;
        this.pturn = new PIDController(0.1,0,0);
        this.pforward = new PIDController(0.1, 0,0);
        
        pturn.setSetpoint(desiredAngle);
        pforward.setSetpoint(desiredDistance);
    }

    @Override
    public void execute() {
        turn = (desiredAngle - camera.getYaw());
        forward = -(desiredDistance - camera.getDistance());
        var turnSpeed = pturn.calculate(turn);
        var driveSpeed = pforward.calculate(forward);


        // questionable?
        // swerve.applyRequest(() -> swerveController
        //     .withSpeeds(new ChassisSpeeds(driveSpeed, strafe, turnSpeed))
        //     .withDriveRequestType(DriveRequestType.Velocity)
        //     .withSteerRequestType(SteerRequestType.MotionMagic));
        swerve.applyRequest(() -> swerveController
            .withVelocityX(driveSpeed)
            .withVelocityY(strafe)
            .withRotationalRate(turnSpeed));
    }

    @Override
    public void end(boolean isFinished) {
        swerve.applyRequest(() -> swerveController
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(desiredAngle - camera.getYaw()) <= 4 && Math.abs(desiredDistance - camera.getDistance()) <= 0.2) {
            return true;
        }
        return false;
    }
}
