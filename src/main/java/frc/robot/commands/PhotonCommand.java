// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// import java.time.zone.ZoneRulesException;
// import java.util.EmptyStackException;
// import java.util.List;
// import java.util.NoSuchElementException;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.PhotonConsts;
// import frc.robot.States.ReefPosition;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.PhotonSubsystem;

// public class PhotonCommand extends Command{
//     private final PhotonSubsystem camera;
//     private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//     private final CommandSwerveDrivetrain drivetrain;
    
//     private Pose2d targetPose;
//     private double xTarget;
//     private double yTarget;
//     private double rTarget;

//     //private final List<Integer> reefTargets;

//     private final PIDController xController;
//     private final PIDController yController;
//     private final PIDController theataController;

//     private Transform3d tagLocation; 

//     private double xSpeed;
//     private double ySpeed;
//     private double theataSpeed;

//     private ReefPosition reefPosition;

//     public PhotonCommand(PhotonSubsystem camera, CommandSwerveDrivetrain drive, ReefPosition reefPosition) {
//         this.camera = camera;
//         this.drivetrain = drive;

//         //reefTargets = PhotonConsts.VALID_REEF_IDS;
//         this.reefPosition = reefPosition;
        
//         xController = new PIDController(2, 0, 0);
//         yController = new PIDController(2, 0, 0);
//         theataController = new PIDController(0.15, 0,0 );
//         theataController.enableContinuousInput(-Math.PI, Math.PI);

//         addRequirements(camera);
//     }

//     @Override
//     public void initialize() { 
//         targetPose = camera.getRobotPoseForNearestReefAprilTag(drivetrain.getState().Pose, reefPosition);

//        xTarget = targetPose.getX();
//        yTarget = targetPose.getY();
//        rTarget = targetPose.getRotation().getRadians();

//        xController.setSetpoint(xTarget);
//        yController.setSetpoint(yTarget);
//        theataController.setSetpoint(rTarget);

//         xController.setTolerance(0.02);
//         yController.setTolerance(0.02);
//         theataController.setTolerance(0.02);
//     }

//     @Override
//     public void execute() {
//         Pose2d currentRobotPose = drivetrain.getState().Pose;

//         double visionTimestamp = camera.getLatestResultTimestamp();
//         //boolean validTarget = PhotonConsts.VALID_REEF_IDS.contains(camera.getTagID());

//         double currentX = currentRobotPose.getX();
//         double currentY = currentRobotPose.getY();
//         double currentR = currentRobotPose.getRotation().getRadians();

//         double xSpeed = xController.calculate(currentX);
//         double ySpeed = yController.calculate(currentY);
//         double theataSpeed = theataController.calculate(currentR);

//         SmartDashboard.putNumber("X Speed", xSpeed);
//         SmartDashboard.putNumber("Y Speed", ySpeed);
//         SmartDashboard.putNumber("THEATA Speed", theataSpeed);

//             //do these if need a clamp ofc
//             // MathUtil.clamp(
//             //     xController.calculate(currentX), -MAX, MAX);

//         drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
//                 .withVelocityY(ySpeed).withRotationalRate(theataSpeed)).execute();
//     }

//     @Override
//     public void end(boolean inFinished) {
//         drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
//         .withVelocityY(0).withRotationalRate(0)).execute();
//     }

//     @Override
//     public boolean isFinished() {
//         return xController.atSetpoint() && yController.atSetpoint() && theataController.atSetpoint();
//     }
// // }