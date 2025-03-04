package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Utilities;
import frc.robot.Constants.PhotonConsts;
import frc.robot.States.ClimbStates;
import frc.robot.States.SuckStates;
import frc.robot.States.ElevatorStates;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ElevatorCommandHandler;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.commands.PhotonCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

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
    private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();

    //subsystems and commands init
    private final PrimaryElevatorSubsystem primaryElevatorSubsystem = new PrimaryElevatorSubsystem();
    private final InnerElevatorSubsystem innerElevatorSubsystem = new InnerElevatorSubsystem();
    private final IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();

    //goHome = new ElevatorCommandHandler(primaryElevatorSubsystem, innerElevatorSubsystem, intakeArmSubsystem, ElevatorStates.HOME_ABS);

    private final PhotonSubsystem photonSubsystem = new PhotonSubsystem(PhotonConsts.CAM_NAMES, PhotonConsts.CAM_TO_ROBOT_TRANSFORMS, drivetrain);
    private final PhotonCommand photonCommand = photonSubsystem.getCommands();

    //logging vars
    private boolean isTrackingAprilTag = false;

    public RobotContainer() {
        //configureDriveBindings();
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

    private void configureSYSTests() {

    }

    private void configureAutos() {
        // register all auto commands
        NamedCommands.registerCommand("Wait Command", new WaitCommand(2));
        m_chooser.addOption("Straight PP Test ", new PathPlannerAuto("Straight Test"));
        SmartDashboard.putData("Auto Chooser", m_chooser);
    }

    private void configureDriver1Commands() {
        //should not deal with states in subsystem but only in command
        //driver1.a().onTrue(intakeArmCommand.setSuck(SuckStates.INTAKE));
        //driver1.b().onTrue(intakeArmCommand.setSuck(SuckStates.FEED_OUT));

        driver1.x().toggleOnTrue(
            photonCommand.goInFrontOfTag(1)
                .beforeStarting(() -> {
                    isTrackingAprilTag = true;
                    SmartDashboard.putBoolean("Tracking AprilTag", isTrackingAprilTag);
                })
                .finallyDo(() -> {
                    isTrackingAprilTag = false;
                    SmartDashboard.putBoolean("Tracking AprilTag", isTrackingAprilTag);
                })
        );


        //driver1.pov(0).onTrue(new ElevatorCommandHandler(primaryElevatorSubsystem, innerElevatorSubsystem, intakeArmSubsystem, ElevatorStates.HOME_REL));
        driver1.pov(90).onTrue(new ElevatorCommandHandler(primaryElevatorSubsystem, innerElevatorSubsystem, intakeArmSubsystem, ElevatorStates.L2_REL));
        driver1.pov(180).onTrue(new ElevatorCommandHandler(primaryElevatorSubsystem, innerElevatorSubsystem, intakeArmSubsystem, ElevatorStates.L3_REL));
        driver1.pov(270).onTrue(new ElevatorCommandHandler(primaryElevatorSubsystem, innerElevatorSubsystem, intakeArmSubsystem, ElevatorStates.L4_REL));
        driver1.pov(-1).onTrue(new ElevatorCommandHandler(primaryElevatorSubsystem, innerElevatorSubsystem, intakeArmSubsystem, ElevatorStates.HOME_REL));
        //driver1.pov(90).toggleOnTrue(primaryElevatorCommand.set(PrimaryElevatorStates.L1));
        //driver1.pov(90).toggleOnTrue(innerElevatorCommand.set(InnerElevatorStates.L1));

        //driver1.pov(180).toggleOnTrue(primaryElevatorCommand.set(PrimaryElevatorStates.L3));
        //driver1.pov(180).toggleOnTrue(innerElevatorCommand.set(InnerElevatorStates.L3));

        //driver1.pov(0).toggleOnTrue(primaryElevatorCommand.set(PrimaryElevatorStates.HOME));
        //driver1.pov(0).toggleOnTrue(innerElevatorCommand.set(InnerElevatorStates.HOME));
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}