package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class alignToPose extends Command {
    private final CommandSwerveDrivetrain drivetrain; 
    private final SwerveRequest.RobotCentric robotCentricDrive = 
     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Pose2d goalPose; 

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController theataController;

    public alignToPose(CommandSwerveDrivetrain drivetrain, Pose2d goalPose2d) {
        this.drivetrain = drivetrain; 
        this.goalPose = goalPose2d;

        xController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);
        theataController = new PIDController(0, 0, 0);
    }

    @Override
    public void initialize() {
        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        theataController.setSetpoint(goalPose.getRotation().getDegrees());

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        theataController.setTolerance(0.1);

        theataController.enableContinuousInput(180, -180);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double theataSpeed = theataController.calculate(currentPose.getRotation().getDegrees());

        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
                 .withVelocityY(ySpeed).withRotationalRate(theataSpeed)).execute();
    }

    @Override
    public void end(boolean isFinished) {
        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
                 .withVelocityY(0).withRotationalRate(0)).execute();
    }
    
    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && theataController.atSetpoint();
    }
}
