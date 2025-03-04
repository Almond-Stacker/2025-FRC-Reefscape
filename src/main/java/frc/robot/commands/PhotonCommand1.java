// package frc.robot.commands;

// import java.util.NoSuchElementException;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.PhotonConsts;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.PhotonSubsystem;

// public class PhotonCommand1 extends Command {
//     private final PhotonSubsystem photon;
//     private final CommandSwerveDrivetrain drivetrain;
//     private final SwerveRequest.RobotCentric robotCentricDrive; 

//     //calculate the PID values
//     private final ProfiledPIDController xController;
//     private final ProfiledPIDController yController;
//     private final ProfiledPIDController rController;

//     private int targetAprilTagID;
//     private boolean availablePose = false;
//     private boolean noTargetFound;
//     private boolean withinDesiredRange;
    

//     private Pose2d aprilTagPose;
    
//     private double xSpeed;
//     private double ySpeed;
//     private double rSpeed;

//     //implement swerve drive subsystem

//     public PhotonCommand1(PhotonSubsystem photonSubsystem, CommandSwerveDrivetrain drivetrain) {
//         this.photon = photonSubsystem;
//         this.drivetrain = drivetrain;
//         this.robotCentricDrive = new RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

//         this.noTargetFound = false;
//         this.withinDesiredRange = false;

//         xController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
//         yController = new ProfiledPIDController(0.5, 0, 0, PhotonConsts.translationConstraints);
//         rController = new ProfiledPIDController(2, 0, 0, PhotonConsts.rotationConstraints);

//         rController.enableContinuousInput(-Math.PI, Math.PI);
//     }

//     @Override
//     public void initialize() {
//         try {
//             targetAprilTagID = photon.getClosestTarget().get()[0].intValue();
//         } catch(NoSuchElementException error) {
//             noTargetFound = true;
//         }

//     }

//     @Override
//     public void execute() {

//     }

//     @Override
//     public void end(boolean isFinished) {
//         drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(0)
//             .withVelocityY(0)
//             .withVelocity);
//     }

//     @Override
//     public boolean isFinished() {
//         if(noTargetFound || withinDesiredRange) {
//             return true;
//         }
//         return false;
//     }
// }