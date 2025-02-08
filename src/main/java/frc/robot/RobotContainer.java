package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Utilities;
import frc.robot.States.InnerElevatorStates;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.commands.InnerElevatorCommand;
import frc.robot.commands.PrimaryElevatorCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevatorSubsystem;
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
    private final AutoChooser autoChooser = new AutoChooser();

    //subsystems and commands init
    private final PrimaryElevatorSubsystem primaryElevatorSubsystem = new PrimaryElevatorSubsystem();
    private final PrimaryElevatorCommand primaryElevatorCommand = primaryElevatorSubsystem.getCommands();

    private final InnerElevatorSubsystem innerElevatorSubsystem = new InnerElevatorSubsystem();
    private final InnerElevatorCommand innerElevatorCommand = innerElevatorSubsystem.getCommands();

    public RobotContainer() {
        configureDriveBindings();
        configureDriver1Commands();
    }
    
    private void configureAutos() {

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

        // reset the field-centric heading on left bumper press
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private void configureSYSTests() {

    }
    
    private void configureDriver1Commands() {
        driver1.pov(90).toggleOnTrue(primaryElevatorCommand.set(PrimaryElevatorStates.L1));
        driver1.pov(90).toggleOnTrue(innerElevatorCommand.set(InnerElevatorStates.L1));

        driver1.pov(180).toggleOnTrue(primaryElevatorCommand.set(PrimaryElevatorStates.L3));
        driver1.pov(180).toggleOnTrue(innerElevatorCommand.set(InnerElevatorStates.L3));

        driver1.pov(0).toggleOnTrue(primaryElevatorCommand.set(PrimaryElevatorStates.HOME));
        driver1.pov(0).toggleOnTrue(innerElevatorCommand.set(InnerElevatorStates.HOME));
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }
}