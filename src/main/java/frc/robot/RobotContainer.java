package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ejml.equation.Sequence;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Utilities;
import frc.robot.Constants.PhotonConsts;
import frc.robot.States.ClimbStates;
import frc.robot.States.ElevatorStates;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ElevatorCommandHandler;
import frc.robot.commands.InnerElevatorCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.PhotonCommand;
import frc.robot.commands.PrimaryElevatorCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

//overall structure of the robot here, no real robot logic
//subsystems, commands, and triggermappings, respectively
public class RobotContainer {
    //generated swerve
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private Command sigma; 
    private SequentialCommandGroup group; 

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

    // ** Choreo Auto set up **//
    private final SendableChooser<Command> m_chooser = AutoBuilder.buildAutoChooser();


    //** Subsystems **//
    private final PrimaryElevatorSubsystem s_primaryElevator = new PrimaryElevatorSubsystem();
    private final InnerElevatorSubsystem s_innerElevator = new InnerElevatorSubsystem();
    private final IntakeArmSubsystem s_intakeArm = new IntakeArmSubsystem();
    private final ClimbSubsystem s_climb = new ClimbSubsystem();

    private final PhotonSubsystem s_photon = new PhotonSubsystem(PhotonConsts.CAM_NAMES, PhotonConsts.CAM_TO_ROBOT_TRANSFORMS, drivetrain);

    //** Command Handlers **/
    private final ClimbCommand ch_climb = s_climb.getCommand();
    private final PhotonCommand ch_photon = s_photon.getCommands();
    private final ElevatorCommandHandler elevatorCommandHandler = new ElevatorCommandHandler(s_primaryElevator, s_innerElevator, s_intakeArm);
    private final IntakeArmCommand intakeArmCommmand = s_intakeArm.getCommands();

    public RobotContainer() {
        configureDriveBindings();
        configureDriver1Commands();
        configureAutos();
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

    private void configureSYSTests() { 

    }

    private void configureAutos() {
        // register all auto commands
        NamedCommands.registerCommand("Wait Command", new WaitCommand(2));
        
        SmartDashboard.putData("Auto Chooser", m_chooser);
    }

    
    private void configureDriver1Commands() {
        
        driver1.a().onTrue(ch_climb.setClimb(ClimbStates.CLIMB));
        driver1.a().onFalse(ch_climb.setClimb(ClimbStates.STOP));

        driver1.b().onTrue(ch_climb.setClimb(ClimbStates.DROP));
        driver1.b().onFalse(ch_climb.setClimb(ClimbStates.STOP));

        // driver1.leftBumper().onTrue(intakeArmCommand.setIntakeState(States.IntakeStates.INTAKE));
        // driver1.leftBumper().onFalse(intakeArmCommand.setIntakeState(States.IntakeStates.STOP));


        // driver1.rightBumper().onTrue(intakeArmCommand.setIntakeState(States.IntakeStates.FEED_OUT));
        // driver1.rightBumper().onFalse(intakeArmCommand.setIntakeState(States.IntakeStates.STOP));

        driver1.pov(0).onTrue(elevatorCommandHandler.set(ElevatorStates.HOME_ABS));
        driver1.pov(90).onTrue(elevatorCommandHandler.set(ElevatorStates.L2_REL));
        driver1.pov(180).onTrue(elevatorCommandHandler.set(ElevatorStates.L3_REL));
        driver1.pov(270).onTrue(elevatorCommandHandler.set(ElevatorStates.L4_REL));


    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}