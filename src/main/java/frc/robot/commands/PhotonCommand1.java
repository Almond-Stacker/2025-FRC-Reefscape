// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;package frc.robot.commands;

// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import frc.robot.Constants.PhotonConsts;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.PhotonSubsystem;

// //This class simply provideds necessary setpoints on the x-y plane spanning the field,
// //where translation into physical movement it done outside of PhotonCommand (drive command)
// public class PhotonCommand1 extends Command{
//     private final PhotonSubsystem photon;
//     private final CommandSwerveDrivetrain drivetrain;
//     private final SwerveRequest.RobotCentric robotCentricDrive;

//     //calculate the PID values
//     private final ProfiledPIDController xController;
//     private final ProfiledPIDController yController;
//     private final ProfiledPIDController rController;

//     private final double offsetDistance;

//     private Pose3d estimatedPose3d; 
//     private Pose2d estimatedPose2d;
//     private Translation2d tagTranslation;
//     private Rotation2d tagRotation;
//     private Translation2d desiredTranslation;
//     private Pose2d targetPose;

//     private int targetAprilTagID;
//     private boolean validTargetID = false;
//     private boolean availablePose = false;

//     private Pose2d aprilTagPose;
    
//     private double xSpeed;
//     private double ySpeed;
//     private double rSpeed;

//     //implement swerve drive subsystem

//     public PhotonCommand1(PhotonSubsystem photonSubsystem, CommandSwerveDrivetrain drivetrain) {
//         this.photon = photonSubsystem;
//         this.drivetrain = drivetrain;

//         xController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
//         yController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
//         rController = new ProfiledPIDController(2, 0, 0, PhotonConsts.rotationConstraints);

//         offsetDistance = 1.0;
//         xSpeed = 0;
//         ySpeed = 0;
//         rSpeed = 0;

//         robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

//         rController.enableContinuousInput(-Math.PI, Math.PI);
//     }

//     @Override
//     public void execute() {
//         estimatedPose3d = photon.getCollectiveEstimatedPose();
//         if(estimatedPose3d == null) {
//             availablePose = false;
//             //drivetrain.stop();
//             return;
//         }
//         availablePose = true;
//         estimatedPose2d = estimatedPose3d.toPose2d();


//         tagTranslation = aprilTagPose.getTranslation();
//         tagRotation = aprilTagPose.getRotation();

//         desiredTranslation = tagTranslation.plus(
//             new Translation2d(-offsetDistance, new Rotation2d(tagRotation.getRadians()))
//         );

//         targetPose = new Pose2d(desiredTranslation, tagRotation);

//         xSpeed = xController.calculate(estimatedPose2d.getX(), targetPose.getX());
//         ySpeed = yController.calculate(estimatedPose2d.getY(), targetPose.getY());
//         rSpeed = rController.calculate(estimatedPose2d.getRotation().getRadians(), targetPose.getRotation().getRadians());

//         drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rSpeed)).execute();
//         setSmartDashboard();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rSpeed)).execute();
//     } 

//     @Override
//     public boolean isFinished() {
//         if(true) {
//             return true;
//         }
//         return false;
//     }

//     public void setSmartDashboard() {
//         SmartDashboard.putNumber("xSpeed", xSpeed);
//         SmartDashboard.putNumber("ySpeed", ySpeed);
//         SmartDashboard.putNumber("rSpeed", rSpeed);

//         SmartDashboard.putNumber("Aptil Tag Pose Opt X", aprilTagPose.getX());
//         SmartDashboard.putNumber("Aptil Tag Pose Opt Y", aprilTagPose.getY());
//         SmartDashboard.putNumber("Aptil Tag Pose Opt Rotation", aprilTagPose.getRotation().getRadians()); 

//         SmartDashboard.putBoolean("AVAILABLE POSE", availablePose);
//     }
// }

