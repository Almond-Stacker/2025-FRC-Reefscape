package frc.robot.commands.AprilTagCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Utilities;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PhotonConsts;
import frc.robot.States.ElevatorStates;
import frc.robot.States.ReefPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevator;
import frc.robot.subsystems.VisionVersions.Vision;
import frc.robot.subsystems.VisionVersions.Vision3;

public class PhotonAlignL3 extends Command {
  private final PIDController xController, yController, rotController;
  private final SwerveRequest.RobotCentric robotCentricDrive = 
     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final InnerElevator innerElevatorSubsystem; 

  private final CommandSwerveDrivetrain drivetrain;
  private final Vision3 camera;
  private final double yOffset; 
    private double xOffset;
    private double translationTolerance; 

  private double desiredYaw;

  private Timer dontSeeTagTimer, stopTimer;


  public PhotonAlignL3(CommandSwerveDrivetrain drivetrain, Vision3 camera, boolean isRight, InnerElevator innerElevatorSubsystem) {
    this.innerElevatorSubsystem = innerElevatorSubsystem;

    xController = new PIDController(2, Constants.PhotonConsts.KI_TRANSLATION, Constants.PhotonConsts.KD_TRANSLATION);  // Vertical movement
    yController = new PIDController(2, Constants.PhotonConsts.KI_TRANSLATION, Constants.PhotonConsts.KD_TRANSLATION);  // Horitontal movement
    rotController = new PIDController(0.04, Constants.PhotonConsts.KI_ROTATION, Constants.PhotonConsts.KD_ROTATION);  // Rotation
    this.drivetrain = drivetrain;
    this.camera = camera; 
    if(isRight) {
      this.yOffset = ReefPosition.RIGHT.yOffset;
    } else {
      this.yOffset = ReefPosition.LEFT.yOffset;
    }
    
   // addRequirements(drivetrain, camera);
    rotController.enableContinuousInput(0, 360);
    addRequirements(drivetrain, camera);
  }

  @Override
  public void initialize() {
    if(innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.L3)) {
        xOffset = PhotonConsts.CENTER_TO_TAG_DELTA_X_L3;
        translationTolerance = 0.04;
    } else {
        xOffset = PhotonConsts.CENTER_TO_TAG_DELTA_X_DEFAULT;
        translationTolerance = 0.04;
    }

    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.desiredYaw = camera.getYaw();

    rotController.setSetpoint(this.desiredYaw);
    SmartDashboard.putNumber("Desired Yaw", this.desiredYaw);

    rotController.setTolerance(PhotonConsts.ROTATIONAL_TOLERANCE);

    xController.setSetpoint(xOffset);
   // xController.setTolerance(PhotonConsts.TRANSLATIONAL_TOLERANCE);
    xController.setTolerance(translationTolerance);


    yController.setSetpoint(yOffset);
   // yController.setTolerance(PhotonConsts.TRANSLATIONAL_TOLERANCE);
   yController.setTolerance(translationTolerance);


  }

  @Override
  public void execute() {
    if (camera.getTargetSeen()) {
      this.dontSeeTagTimer.reset();
      Transform3d positions = camera.getBestTagToCamera();
      double diffY = Math.max(-0.19, positions.getY() - yOffset);
      double diffX = Math.max(-0.19, positions.getX() - xOffset);
      double addedX = Math.max(0, Math.min(0.6, Math.log(diffX + 0.12) * 0.8 + 0.5));
      double addedY = Math.max(0, Math.min(Math.log(diffY + 0.12) * 0.8 + 0.5, 0.6));
    
      double xSpeed = -xController.calculate(positions.getX() + addedX);
      double ySpeed = -yController.calculate(positions.getY() + addedY);
      double rotValue = rotController.calculate(Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));

      SmartDashboard.putNumber("X Target Offset", positions.getX());
      SmartDashboard.putNumber("Xspeed", xSpeed);
      SmartDashboard.putNumber("Calculated yaw", Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
      SmartDashboard.putNumber("Rspeed", rotValue);

      drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
        .withVelocityY(ySpeed).withRotationalRate(rotValue)).execute();

      //reset to 0 if not at desired position
      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
        .withVelocityY(0).withRotationalRate(0)).execute();    
    }

    SmartDashboard.putNumber("Pose Validation Timer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
      .withVelocityY(0).withRotationalRate(0)).execute();
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.PhotonConsts.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.PhotonConsts.POSE_VALIDATION_TIME);
  }
}