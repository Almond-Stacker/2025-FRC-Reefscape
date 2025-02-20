package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonSubsystem;

//This class simply provideds necessary setpoints on the x-y plane spanning the field,
//where translation into physical movement it done outside of PhotonCommand (drive command)
public class PhotonCommand {
    
    private PhotonSubsystem photon;
    private CommandSwerveDrivetrain drivetrain;

    //calculate the PID values
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController rController;

    private int targetAprilTagID;
    private Optional<Pose3d> aprilTagPoseOpt;
    private boolean validTargetID = false;
    private boolean availablePose = false;
    
    private double xSpeed;
    private double ySpeed;
    private double rSpeed;

    //implement swerve drive subsystem

    public PhotonCommand(PhotonSubsystem photonSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.photon = photonSubsystem;
        this.drivetrain = drivetrain;

        xController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
        yController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
        rController = new ProfiledPIDController(2, 0, 0, PhotonConsts.rotationConstraints);

        xSpeed = 0;
        ySpeed = 0;
        rSpeed = 0;

        rController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command goInFrontOfTag(int targetAprilTagID) {
        setTarget(targetAprilTagID);

        return new RunCommand(() -> {
            Optional<EstimatedRobotPose> estimatedPoseOpt = photon.getCollectiveEstimatedPose();

            if(estimatedPoseOpt.isEmpty()) {
                availablePose = false;
                SmartDashboard.putBoolean("AVAILABLE POSE", availablePose);
                //drivetrain.stop();
                return;
            }
            availablePose = true;
            SmartDashboard.putBoolean("AVAILABLE POSE", availablePose);
            Pose2d estimatedPose = estimatedPoseOpt.get().estimatedPose.toPose2d();

            if(aprilTagPoseOpt.isEmpty()) {
                validTargetID = false;
                SmartDashboard.putBoolean("VALID ID", validTargetID);
                return;
            }
            SmartDashboard.putBoolean("VALID ID", validTargetID);
            validTargetID = true;
            
            Pose2d aprilTagPose = aprilTagPoseOpt.get().toPose2d();

            double offsetDistance = 2.0; // meters
            Translation2d tagTranslation = aprilTagPose.getTranslation();
            Rotation2d tagRotation = aprilTagPose.getRotation();

            Translation2d desiredTranslation = tagTranslation.plus(
                new Translation2d(-offsetDistance, new Rotation2d(tagRotation.getRadians()))
            );

            Pose2d targetPose = new Pose2d(desiredTranslation, tagRotation);

            xSpeed = xController.calculate(estimatedPose.getX(), targetPose.getX());
            ySpeed = yController.calculate(estimatedPose.getY(), targetPose.getY());
            rSpeed = rController.calculate(estimatedPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            SmartDashboard.putNumber("xSpeed", targetPose.getX());
        SmartDashboard.putNumber("ySpeed", targetPose.getY());
        SmartDashboard.putNumber("rSpeed", targetPose.getRotation().getRadians());

        }, drivetrain).finallyDo(() -> drivetrain.stop());
    }

    public void setTarget(int targetAprilTagID) {
        this.targetAprilTagID = targetAprilTagID;
        aprilTagPoseOpt = PhotonConsts.aprilTagFieldLayout.getTagPose(targetAprilTagID);
    }

    public void setSmartDashboard() {
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rSpeed", rSpeed);
    }

    public PhotonSubsystem getPhotonSubsystem() {
        return photon;
    }

}