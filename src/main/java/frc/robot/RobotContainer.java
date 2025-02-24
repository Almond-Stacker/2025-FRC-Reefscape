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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.lib.util.Utilities;
import frc.robot.CommandFactory.AlgaeIntakeCommandFactory;
import frc.robot.CommandFactory.CombinationCommandFactory;
import frc.robot.CommandFactory.InnerElevatorCommandFactory;
import frc.robot.CommandFactory.IntakeArmCommandFactory;
import frc.robot.CommandFactory.PrimaryElevatorCommandFactory;
import frc.robot.Constants.PrimaryElevator;
import frc.robot.States.InnerElevatorStates;
import frc.robot.States.IntakeArmStates;
import frc.robot.States.IntakeStates;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.commands.InnerElevatorCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.PrimaryElevatorCommand;
import frc.robot.commands.sigma;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class RobotContainer {
    // Initalize Subsytems and subsystem controllers 
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driver0 = new CommandXboxController(0);
    private final CommandXboxController driver1 = new CommandXboxController(1);
    public final  CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    /* Path follower */
    // Choreo set up 
    private final AutoFactory autoFactory = drivetrain.createAutoFactory();
    private final AutoRoutines autoRoutines = new AutoRoutines(autoFactory);
    private final AutoChooser autoChooser = new AutoChooser();

    //private final PhotonSubsystem camera0 = new PhotonSubsystem(Constants.Photon.camera0.cameraName,  Constants.Photon.camera0.cameraHeight, Constants.Photon.camera0.cameraPitch, PhotonStates.driveTag4);

    /** Subsystems **/
    private final PrimaryElevatorSubsystem s_PrimaryElevator = new PrimaryElevatorSubsystem();
    private final InnerElevatorSubsystem s_InnerElevator = new InnerElevatorSubsystem();
    private final IntakeArmSubsystem s_intakeArm  = new IntakeArmSubsystem();

    /** Commands **/
    private final PrimaryElevatorCommand ch_primaryElevator = s_PrimaryElevator.getCommand();
    private final InnerElevatorCommand ch_innerElevator = s_InnerElevator.getCommand();
    private final IntakeArmCommand ch_intakeArm = s_intakeArm.getCommand();

    private final SequentialCommandGroup c_home = new SequentialCommandGroup(
        ch_primaryElevator.set(PrimaryElevatorStates.HOME),
        ch_innerElevator.set(InnerElevatorStates.HOME),
        ch_intakeArm.setAngle(IntakeArmStates.HOME),
        ch_intakeArm.setIntakeSpeed(IntakeStates.STOP)
    );

    public RobotContainer() {
        configureAutos();
        configureDriveBindings();
        configureSYSTests();  
        configureDriver1Commands();  
    }

    private void configureAutos() {
        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("SimplePath1", autoRoutines::simplePathAuto1);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Utilities.polynomialAccleration(driver0.getLeftY()) * -MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(Utilities.polynomialAccleration(driver0.getLeftX()) * -MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(Utilities.polynomialAccleration(driver0.getRightX()) * -MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

         driver0.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
        driver0.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))));

        // // reset the field-centric heading on left bumper press
         driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver0.y().onTrue(i);
    }
    

    private void configureSYSTests() {
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver0.back().and(driver0.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver0.back().and(driver0.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver0.start().and(driver0.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver0.start().and(driver0.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    private void configureDriver1Commands() {
        driver1.rightTrigger().onTrue(c_coralIntake);
        driver1.rightTrigger().onFalse(c_armStop);

        driver1.leftTrigger().onTrue(c_coralFeedOut);
        driver1.leftTrigger().onFalse(c_armStop);

        driver1.a().toggleOnTrue(c_preIntake);
        driver1.x().toggleOnTrue(c_intake);
        driver1.y().toggleOnTrue(c_feedOutScore);
        driver1.pov(90).toggleOnTrue(c_scoreL1);
        driver1.pov(180).toggleOnTrue(c_scoreL2);
        driver1.pov(270).toggleOnTrue(c_scoreL3);
        driver1.pov(0).toggleOnTrue(c_home);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
