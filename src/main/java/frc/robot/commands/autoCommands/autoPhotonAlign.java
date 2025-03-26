package frc.robot.commands.autoCommands;

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
import frc.robot.States.ReefPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionVersions.Vision;
import frc.robot.subsystems.VisionVersions.Vision3;

public class autoPhotonAlign extends Command {
  private final PIDController xController, yController, rotController;
  private final SwerveRequest.RobotCentric robotCentricDrive = 
     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision3 camera;
  private final double yOffset; 
  private double desiredYaw;
  private double startTime; 

  private Timer dontSeeTagTimer, stopTimer;


  public autoPhotonAlign(CommandSwerveDrivetrain drivetrain, Vision3 camera, boolean isRight) {
    xController = new PIDController(2.1, Constants.PhotonConsts.KI_TRANSLATION, Constants.PhotonConsts.KD_TRANSLATION);  // Vertical movement
    yController = new PIDController(2.1, Constants.PhotonConsts.KI_TRANSLATION, Constants.PhotonConsts.KD_TRANSLATION);  // Horitontal movement
    rotController = new PIDController(0.07
    , Constants.PhotonConsts.KI_ROTATION, Constants.PhotonConsts.KD_ROTATION);  // Rotation
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
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.desiredYaw = camera.getYaw();
    this.startTime = Timer.getFPGATimestamp();

    rotController.setSetpoint(this.desiredYaw);
    SmartDashboard.putNumber("Desired Yaw", this.desiredYaw);

    rotController.setTolerance(PhotonConsts.ROTATIONAL_TOLERANCE);

    xController.setSetpoint(PhotonConsts.CENTER_TO_TAG_DELTA_X);
    xController.setTolerance(PhotonConsts.TRANSLATIONAL_TOLERANCE);

    yController.setSetpoint(yOffset);
    yController.setTolerance(PhotonConsts.TRANSLATIONAL_TOLERANCE);

  }

  @Override
  public void execute() {
    if (camera.getTargetSeen()) {
      this.dontSeeTagTimer.reset();
      Transform3d positions = camera.getBestTagToCamera();
      double xSpeed = -xController.calculate(positions.getX());
      double ySpeed = -yController.calculate(positions.getY());
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
        System.out.println("sigma nga") ;  
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
    System.out.println("sigma");
    if(Timer.getFPGATimestamp() - startTime > 4) {
        return true;
    }

    if(xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint()) {
        return true; 
    }
    return false; 
  }
}