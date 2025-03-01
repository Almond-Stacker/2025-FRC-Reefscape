// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import frc.lib.util.Utilities;

import frc.robot.Constants.PrimaryElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.States.ClimbStates;
import frc.robot.States.InnerElevatorStates;
import frc.robot.States.IntakeArmStates;
import frc.robot.States.IntakeStates;
import frc.robot.States.PrimaryElevatorStates;

import frc.robot.commands.ClimbCommand;
import frc.robot.commands.InnerElevatorCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.PrimaryElevatorCommand;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
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
    private final ClimbSubsystem s_climb = new ClimbSubsystem();

    /** Commands **/
    private final PrimaryElevatorCommand ch_primaryElevator = s_PrimaryElevator.getCommand();
    private final InnerElevatorCommand ch_innerElevator = s_InnerElevator.getCommand();
    private final IntakeArmCommand ch_intakeArm = s_intakeArm.getCommand();
    private final ClimbCommand ch_climb = s_climb.getCommand();

    private final Command c_startClimb = ch_climb.setClimb(ClimbStates.CLIMB);
    private final Command c_dropClimb = ch_climb.setClimb(ClimbStates.DROP);
    private final Command c_stopClimb = ch_climb.setClimb(ClimbStates.STOP);

    private final SequentialCommandGroup c_home = new SequentialCommandGroup(
        ch_primaryElevator.set(PrimaryElevatorStates.STARTING_POSITION),
        ch_innerElevator.set(InnerElevatorStates.STARTING_POSITION),
        ch_intakeArm.setAngle(IntakeArmStates.STARTING_POSITION),
        ch_intakeArm.setIntakeSpeed(IntakeStates.STOP)
    );

    public RobotContainer() {
        configureAutos();
        configureDriveBindings();
        configureSYSTests();  
        configureDriver1Commands();  
        //driver1.a().whileTrue(new InstantCommand(() -> s_PrimaryElevator.setElevatorSpeed(0.05)));
       // driver1.a().whileFalse  (new InstantCommand(() -> s_PrimaryElevator.setElevatorSpeed(0)));
        driver1.b().onTrue(new InstantCommand(() -> s_climb.setClimb(0.06)));
        driver1.b().onFalse(new InstantCommand(() -> s_climb.setClimb(0)));
        driver1.y().onTrue(new InstantCommand(() -> s_climb.setClimb(-0.06)));
        driver1.y().onFalse(new InstantCommand(() -> s_climb.setClimb(0)));
        driver1.x().onTrue(new InstantCommand(() -> s_climb.setClimb(-1)));
        driver1.x().onFalse(new InstantCommand(() -> s_climb.setClimb(0)));
    }

    private void configureAutos() {
        autoChooser.addRoutine("SimplePath2", autoRoutines::simplePathAuto2);

        autoChooser.addRoutine("SimplePath13", autoRoutines::simplePathAuto3);

        autoChooser.addRoutine("SimplePath14", autoRoutines::simplePathAuto4);


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

        //  driver0.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
        // driver0.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))));

        // // // reset the field-centric heading on left bumper press
      driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // driver0.y().onTrue(i);
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

    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
