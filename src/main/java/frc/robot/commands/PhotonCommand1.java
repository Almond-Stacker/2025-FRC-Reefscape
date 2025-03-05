package frc.robot.commands;

import java.util.NoSuchElementException;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhotonConsts;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonSubsystem;

public class PhotonCommand1 extends Command {
    private final PhotonSubsystem photon;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric robotCentricDrive; 

    //calculate the PID values
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController rController;

    private int targetAprilTagID;
    private boolean availablePose = false;
    private boolean noTargetFound;
    private boolean withinDesiredRange;
    private boolean cancelCommand; 
    

    private Pose2d aprilTagPose;
    
    private double xSpeed;
    private double ySpeed;
    private double rSpeed;

    private int counter; 

    private double currentTime;
    private double lastTime;
    

    //implement swerve drive subsystem

    public PhotonCommand1(PhotonSubsystem photonSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.photon = photonSubsystem;
        this.drivetrain = drivetrain;
        this.robotCentricDrive = new RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.noTargetFound = false;
        this.withinDesiredRange = false;
        this.counter = 0; 

        lastTime = Timer.getFPGATimestamp();

        xController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
        yController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
        rController = new ProfiledPIDController(2, 0, 0, PhotonConsts.rotationConstraints);

        rController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        
            
    }

    @Override
    public void execute() {
        try {
            targetAprilTagID = photon.getClosestTarget().get()[0].intValue();

        } catch(NoSuchElementException error) {
            noTargetFound = true;
        }
        Pose3d estimatedPose3d = photon.getCollectiveEstimatedPose();

        if(estimatedPose3d == null) {
            noTargetFound = true;
            return; 
        }

        counter = 0; 

        Pose2d estimatedPose2d = estimatedPose3d.toPose2d();
        double targetDistance = estimatedPose2d.getTranslation().getNorm();

        SmartDashboard.putNumber("Aptil Tag Pose Opt X", aprilTagPose.getX());
        SmartDashboard.putNumber("Aptil Tag Pose Opt Y", aprilTagPose.getY());
        SmartDashboard.putNumber("Aptil Tag Pose Opt Rotation", aprilTagPose.getRotation().getRadians());


        double offsetDistance = 1.0; // meters
        Translation2d tagTranslation = aprilTagPose.getTranslation();
        Rotation2d tagRotation = aprilTagPose.getRotation();

        Translation2d desiredTranslation = tagTranslation.plus(
            new Translation2d(-offsetDistance, new Rotation2d(tagRotation.getRadians()))
        );

        Pose2d targetPose = new Pose2d(desiredTranslation, tagRotation);

        //calculate difference from target
        if(desiredTranslation.getNorm() < PhotonConsts.DESIRED_RANGE) {
            withinDesiredRange = true;
            return;
        }

        xSpeed = xController.calculate(estimatedPose2d.getX(), targetPose.getX());
        ySpeed = yController.calculate(estimatedPose2d.getY(), targetPose.getY());
        rSpeed = rController.calculate(estimatedPose2d.getRotation().getRadians(), targetPose.getRotation().getRadians());

        drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rSpeed)).execute();

        currentTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean isFinished) {
        drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)).execute();
    }

    @Override
    public boolean isFinished() {
        if(noTargetFound && currentTime - lastTime > PhotonConsts.TIMEOUT_COMMAND) {
            return true;
        }
        
        if(withinDesiredRange) {
            return true;
        }
        
        return false;
    }
}