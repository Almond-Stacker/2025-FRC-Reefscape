
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
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Utilities;

import frc.robot.commands.ClimbCommand;
import frc.robot.commands.InnerElevatorCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.PrimaryElevatorCommand;
import frc.robot.generated.TunerConstants;
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

    //subsystems and commands init
    // private final PrimaryElevatorSubsystem primaryElevatorSubsystem = new PrimaryElevatorSubsystem();
    // private final PrimaryElevatorCommand primaryElevatorCommand = primaryElevatorSubsystem.getCommands();

    // private final InnerElevatorSubsystem innerElevatorSubsystem = new InnerElevatorSubsystem();
    // private final InnerElevatorCommand innerElevatorCommand = innerElevatorSubsystem.getCommands();

     private final IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
     private final IntakeArmCommand intakeArmCommand = intakeArmSubsystem.getCommands();

    // private final ElevatorCommandHandler elevatorCommandHandler = 
    // new ElevatorCommandHandler(
    //     primaryElevatorSubsystem, innerElevatorSubsystem, intakeArmSubsystem
    // );
    

    // private final PhotonSubsystem photonSubsystem = new PhotonSubsystem(PhotonConsts.CAM_NAMES, PhotonConsts.CAM_TO_ROBOT_TRANSFORMS, drivetrain);
    // private final PhotonCommand photonCommand = photonSubsystem.getCommands();

    // //logging vars
    // private boolean isTrackingAprilTag = false;

    //** Subsystems **//
    private final PrimaryElevatorSubsystem s_primaryElevator = new PrimaryElevatorSubsystem();
    private final InnerElevatorSubsystem s_innerElevator = new InnerElevatorSubsystem();
    private final IntakeArmSubsystem s_intakeArm = new IntakeArmSubsystem();
    private final ClimbSubsystem s_climb = new ClimbSubsystem();

    //** Command Handlers **/
    private final ClimbCommand ch_climb = s_climb.getCommand();
    private final ElevatorCommandHandler ch_elevator = new ElevatorCommandHandler(s_primaryElevator, s_innerElevator, s_intakeArm);

    public RobotContainer() {
        configureAutos();
        configureDriveBindings();
        configureSYSTests();  
    }

    private void configureAutos() {
        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("SimplePath1", autoRoutines::simplePathAuto1);
    autoChooser.addRoutine("What the sigma ", autoRoutines::simplePathAuto12);
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
    }
    

    private void configureSYSTests() { 

    }

    private void configureAutos() {
        // register all auto commands
        NamedCommands.registerCommand("Wait Command", new WaitCommand(2));

        // add all auto paths
        autoChooser.addRoutine("Straight Path Short", autoRoutines::ShortTest);
        autoChooser.addRoutine("Diagonal Path", autoRoutines::DiagonalTest);
        autoChooser.addRoutine("Curve Path", autoRoutines::CurveTest);
        autoChooser.addRoutine("Straight Path Long", autoRoutines::StraightTest);
        autoChooser.addRoutine("Game Auto", autoRoutines::GameAuto1);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    
    private void configureDriver1Commands() {
        driver1.a().onTrue(ch_climb.setClimb(ClimbStates.CLIMB));
        driver1.a().onFalse(ch_climb.setClimb(ClimbStates.STOP));

        driver1.b().onTrue(ch_climb.setClimb(ClimbStates.DROP));
        driver1.b().onFalse(ch_climb.setClimb(ClimbStates.STOP));

        driver1.leftBumper().onTrue(intakeArmCommand.setIntakeState(States.IntakeStates.INTAKE));
        driver1.leftBumper().onFalse(intakeArmCommand.setIntakeState(States.IntakeStates.STOP));

        driver1.rightBumper().onTrue(intakeArmCommand.setIntakeState(States.IntakeStates.FEED_OUT));
        driver1.rightBumper().onFalse(intakeArmCommand.setIntakeState(States.IntakeStates.STOP));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
