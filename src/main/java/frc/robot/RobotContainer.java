// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants;
import frc.robot.States.PhotonStates;
import frc.robot.commands.positionRelativeToAprilTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonSubsystem;

public class RobotContainer {
    // Initalize Subsytems and subsystem controllers 
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driver0 = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    
    /* Path follower */
    // Choreo set up 
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    private final PhotonSubsystem camera0 = new PhotonSubsystem(Constants.Photon.camera0.cameraName,  Constants.Photon.camera0.cameraHeight, Constants.Photon.camera0.cameraPitch, PhotonStates.driveTag4);



    // Initailize commands
    private final positionRelativeToAprilTag tag4Pos0 = new positionRelativeToAprilTag(camera0, PhotonStates.driveTag4);

    private final SequentialCommandGroup ramTag7 = new SequentialCommandGroup(tag4Pos0, 
                                                        drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(camera0.getForwardOutput())
                                                        .withVelocityY(driver0.getLeftY())
                                                        .withRotationalRate(camera0.getTurnOutput())));


    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureDriveBindings();
    }

    private void configureDriveBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver0.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver0.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver0.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver0.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver0.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))
        ));
        driver0.y().whileTrue(ramTag7);
        driver0.x().toggleOnTrue(drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(-driver0.getLeftY() * MaxSpeed)
                                    .withVelocityY(-driver0.getLeftX() * MaxSpeed)
                                    .withRotationalRate(-driver0.getRightX() * MaxAngularRate)));
                                    

        driver0.pov(0).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0.5).withVelocityY(0))
        );
        driver0.pov(180).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver0.back().and(driver0.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver0.back().and(driver0.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver0.start().and(driver0.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver0.start().and(driver0.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
