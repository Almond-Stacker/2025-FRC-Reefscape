package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.proto.Photon;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.util.Utilities;

import frc.robot.States.ClimbStates;
import frc.robot.States.ElevatorStates;
import frc.robot.States.IndexStates;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ElevatorCommandHandler;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.PhotonCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BeambreakSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.DriveVelSubsystem;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
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
    
    private SendableChooser<Command> m_chooser;

    //** Subsystems **//
    private final PrimaryElevatorSubsystem s_primaryElevatorSubsystem = new PrimaryElevatorSubsystem();
    private final InnerElevatorSubsystem s_innerElevatorSubsystem = new InnerElevatorSubsystem();
    private final ArmSubsystem s_armSubsystem = new ArmSubsystem();
    private final ClimbSubsystem s_climbSubsystem = new ClimbSubsystem();
    private final BeambreakSubsystem s_beambreakSubsystem = new BeambreakSubsystem();
    private final PhotonSubsystem s_grayPhotonVision = new PhotonSubsystem("gray_photon_camera", () -> drivetrain.getRotation3d().getAngle());
    private final PhotonSubsystem s_bluePhotonVision = new PhotonSubsystem("blue_photon_camera", () -> drivetrain.getRotation3d().getAngle());

    //** Command Handlers **//
    private final ElevatorCommandHandler ch_elevatorCommandHandler = new ElevatorCommandHandler(s_primaryElevatorSubsystem, s_innerElevatorSubsystem, s_armSubsystem);
    private final ClimbCommand ch_climbCommand = new ClimbCommand(s_climbSubsystem);
    private final IndexCommand ch_indexCommand = new IndexCommand(s_armSubsystem);

    //** Commands **//
    private final PhotonCommand c_positionToRightPole = new PhotonCommand(s_grayPhotonVision, drivetrain, 0, 0.43, 0.2);
    private final PhotonCommand c_positionToLeftPole = new PhotonCommand(s_bluePhotonVision, drivetrain, 0, 0.5, 0.2);

    private final SequentialCommandGroup c_preIntakeToIntake = new SequentialCommandGroup(
        ch_elevatorCommandHandler.setArmState(ElevatorStates.INTAKE),
        ch_elevatorCommandHandler.setPrimaryElevatorState(ElevatorStates.INTAKE),
        //ch_elevatorCommandHandler.setElevators(ElevatorStates.INTAKE),
        ch_indexCommand.setIndexState(IndexStates.INTAKE),
        new WaitCommand(0.2),
        ch_elevatorCommandHandler.setInnerElevatorState(ElevatorStates.INTAKE),
        new WaitCommand(0.5),
        ch_indexCommand.setIndexState(IndexStates.STOP),
        ch_elevatorCommandHandler.setElevators(ElevatorStates.PRE_INTAKE));
    
    private final SequentialCommandGroup c_score = new SequentialCommandGroup(
        new InstantCommand(() -> s_armSubsystem.setOverride(-0.3, true)),
        ch_indexCommand.setIndexState(IndexStates.OUTTAKE),
        new WaitCommand(0.5),
        new InstantCommand(() -> s_armSubsystem.setOverride(0, false)),
        ch_indexCommand.setIndexState(IndexStates.STOP),
        ch_elevatorCommandHandler.setElevators(ElevatorStates.PRE_INTAKE)
    );

    public RobotContainer() {
        configureDriveBindings();
        configureDriver1Commands();
        configureAuto();
    }

    private boolean checkRun() {
        if(!s_beambreakSubsystem.getBeamBroken() && s_innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.PRE_INTAKE)) {
            return true;
        }
        return false;
    }

    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Utilities.polynomialAccleration(driver0.getLeftY()) * MaxSpeed * 0.4) // Drive forward with negative Y (forward)
                    .withVelocityY(-Utilities.polynomialAccleration(driver0.getLeftX()) * MaxSpeed * 0.4) // Drive left with negative X (left)
                    .withRotationalRate(-Utilities.polynomialAccleration(driver0.getRightX()) * MaxAngularRate * 0.4) // Drive counterclockwise with negative X (left)
            )
        );

        driver0.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver0.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))));
        //driver0.x().whileTrue(spot1);

        // reset the field-centric heading on left bumper press
       driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
       driver0.x().onTrue(c_positionToRightPole);
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(
             new Rotation2d(SwerveRequest.ForwardPerspectiveValue.valueOf(180).value))));
    }

    private void configureDriver1Commands() {
        driver1.pov(0).toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L4));
        driver1.pov(90).toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L3));
        driver1.pov(180).toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L2));
        driver1.pov(270).toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L1));
        driver1.pov(-1).toggleOnTrue(c_preIntakeToIntake.onlyIf(() -> checkRun()));
        driver1.a().toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.PRE_INTAKE));
        //driver1.b().toggleOnTrue(c_preIntakeToIntake.onlyIf(() -> checkRun()));
        driver1.y().onTrue(c_score);

        driver1.rightTrigger().onTrue(ch_climbCommand.setClimbState(ClimbStates.CLIMB));
        driver1.rightTrigger().onFalse(ch_climbCommand.setClimbState(ClimbStates.STOP));

        driver1.leftTrigger().onTrue(ch_climbCommand.setClimbState(ClimbStates.DROP));
        driver1.leftTrigger().onFalse(ch_climbCommand.setClimbState(ClimbStates.STOP));
    }

    private void configureAuto() {
        NamedCommands.registerCommand("Score", c_score);
        NamedCommands.registerCommand("L4", ch_elevatorCommandHandler.setElevators(ElevatorStates.L4));
        NamedCommands.registerCommand("L1", ch_elevatorCommandHandler.setElevators(ElevatorStates.L1));
        NamedCommands.registerCommand("Pre intake", ch_elevatorCommandHandler.setElevators(ElevatorStates.PRE_INTAKE));
        NamedCommands.registerCommand("intake", c_preIntakeToIntake);
        NamedCommands.registerCommand("LineUp", new WaitCommand(0.1));    
        NamedCommands.registerCommand("Intake", ch_indexCommand.setIndexState(IndexStates.INTAKE));
        NamedCommands.registerCommand("Stop", ch_indexCommand.setIndexState(IndexStates.STOP));

        m_chooser = AutoBuilder.buildAutoChooser("1 Coral Straight");
        SmartDashboard.putData(m_chooser);
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public void resetDrive() {
        drivetrain.runOnce(() -> drivetrain.resetRotation(
            new Rotation2d(SwerveRequest.ForwardPerspectiveValue.valueOf(180).value)));
    }
}