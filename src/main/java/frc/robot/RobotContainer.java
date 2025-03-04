package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.util.Utilities;

import frc.robot.Constants.PhotonConsts;

import frc.robot.States.ClimbStates;
import frc.robot.States.ElevatorStates;

import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ElevatorCommandHandler;
import frc.robot.commands.IntakeCommand;

import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

import frc.robot.generated.TunerConstants;


//overall structure of the robot here, no real robot logic
//subsystems, commands, and triggermappings, respectively
public class RobotContainer {
    //generated swerve
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    //bindings and all for swerve control
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driver0 = new CommandXboxController(0);
    private final CommandXboxController driver1 = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //put choreo
    //private final AutoChooser autoChooser = new AutoChooser();
    private SendableChooser<Command> m_chooser;

    //** Subsystems **//
    private final PrimaryElevatorSubsystem s_primaryElevatorSubsystem = new PrimaryElevatorSubsystem();
    private final InnerElevatorSubsystem s_innerElevatorSubsystem = new InnerElevatorSubsystem();
    private final IntakeArmSubsystem s_intakeArmSubsystem = new IntakeArmSubsystem();
    private final ClimbSubsystem s_climbSubsystem = new ClimbSubsystem();

    //** Command Handlers **//
    private final ElevatorCommandHandler ch_elevatorCommandHandler = new ElevatorCommandHandler(s_innerElevatorSubsystem, s_primaryElevatorSubsystem, s_intakeArmSubsystem);
    private final IntakeCommand ch_intakeCommandHandler = new IntakeCommand(s_intakeArmSubsystem);
    private final ClimbCommand ch_climbCommandHandler = new ClimbCommand(s_climbSubsystem);

    //** Commands **//

    //goHome = new ElevatorCommandHandler(primaryElevatorSubsystem, innerElevatorSubsystem, intakeArmSubsystem, ElevatorStates.HOME_ABS);

    //private final PhotonSubsystem photonSubsystem = new PhotonSubsystem(PhotonConsts.CAM_NAMES, PhotonConsts.CAM_TO_ROBOT_TRANSFORMS, drivetrain);
   // private final PhotonCommand photonCommand = photonSubsystem.getCommands();

    //logging vars
   /// private boolean isTrackingAprilTag = false;

    public RobotContainer() {
        m_chooser = AutoBuilder.buildAutoChooser("Blue Bottom Corner");
        SmartDashboard.putData(m_chooser);
        configureDriveBindings();
        configureDriver1Commands();
    }

    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Utilities.polynomialAccleration(driver0.getLeftY()) * -MaxSpeed * 0.4) // Drive forward with negative Y (forward)
                    .withVelocityY(Utilities.polynomialAccleration(driver0.getLeftX()) * -MaxSpeed * 0.4) // Drive left with negative X (left)
                    .withRotationalRate(Utilities.polynomialAccleration(driver0.getRightX()) * -MaxAngularRate * 0.4 ) // Drive counterclockwise with negative X (left)
        ));

        driver0.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver0.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))));

        // reset the field-centric heading on left bumper press
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private void configureDriver1Commands() {
        // driver1.a().onTrue(ch_climbCommandHandler.setClimb(ClimbStates.CLIMB));
        // driver1.a().onFalse(ch_climbCommandHandler.setClimb(ClimbStates.STOP));

        // driver1.b().onTrue(ch_climbCommandHandler.setClimb(ClimbStates.DROP));
        // driver1.b().onFalse(ch_climbCommandHandler.setClimb(ClimbStates.STOP));

        //elevator system
        driver1.pov(-1).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.HOME, true));
        driver1.pov(0).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.HOME, false));
        driver1.pov(90).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L1, false));
        driver1.pov(180).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L2, false));
        driver1.pov(270).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L3, false));

        // driver1.pov(-1).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.HOME));
        // driver1.pov(0).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.HOME));
        // driver1.pov(90).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L1));
        // driver1.pov(180).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L2));
        // driver1.pov(270).onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L4));

        driver1.leftBumper().onTrue(new InstantCommand(() -> {s_innerElevatorSubsystem.setMotorSpeed(0.05);}));
        driver1.leftBumper().onFalse(new InstantCommand(() -> {s_innerElevatorSubsystem.setMotorSpeed(0);}));

        driver1.rightBumper().onTrue(new InstantCommand(() -> {s_innerElevatorSubsystem.setMotorSpeed(-0.05);}));
        driver1.rightBumper().onFalse(new InstantCommand(() -> {s_innerElevatorSubsystem.setMotorSpeed(0);}));

        driver1.leftTrigger().onTrue(new InstantCommand(() -> {s_intakeArmSubsystem.setMotorSpeed(0.05);}));
        driver1.leftTrigger().onFalse(new InstantCommand(() -> {s_intakeArmSubsystem.setMotorSpeed(0);}));

        driver1.rightTrigger().onTrue(new InstantCommand(() -> {s_intakeArmSubsystem.setMotorSpeed(-0.05);}));
        driver1.rightTrigger().onFalse(new InstantCommand(() -> {s_intakeArmSubsystem.setMotorSpeed(0);}));
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}