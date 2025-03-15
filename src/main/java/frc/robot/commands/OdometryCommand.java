package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Utilities;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class OdometryCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain; 
    private final PIDController xController;
    private final double goalXDistance = 0.416;

    private Pose2d currentPose; 

    private final SwerveRequest.RobotCentric 
    robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public OdometryCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        xController = new PIDController(1, 0.0, 0.0);
    }

    @Override
    public void initialize() {
        drivetrain.resetPose(new Pose2d());
    }

    @Override 
    public void execute() {
        currentPose = drivetrain.getState().Pose;
        double distance = currentPose.getTranslation().getNorm();

        double xSpeed = -xController.calculate(distance, goalXDistance);

        drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
                 .withVelocityY(0).withRotationalRate(0)).execute();
    }
}