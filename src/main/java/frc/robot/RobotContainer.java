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
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.Photon1;
import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PrimaryElevator;

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
    public final Photon1 sigma = new Photon1(drivetrain);
    
    private SendableChooser<Command> m_chooser;

    //** Subsystems **//
   
    // private final PhotonSubsystem s_grayPhotonVision = new PhotonSubsystem("gray_photon_camera", () -> drivetrain.getPigeon2().getYaw().getValueAsDouble(), drivetrain);
    // private final PhotonSubsystem s_bluePhotonVision = new PhotonSubsystem("blue_photon_camera", () -> drivetrain.getRotation3d().getAngle(), drivetrain);


    // private PhotonCommand c_positionToRightPole = new PhotonCommand(s_grayPhotonVision, drivetrain, 0, 0.416, 0.17, () -> driver0.getRightY());
    // private PhotonCommand c_positionToLeftPole = new PhotonCommand(s_bluePhotonVision, drivetrain, 0, 0.416, -0.17, () -> driver0.getRightY());

    // private PhotonCommand1 c_positionToRightPole1 = new PhotonCommand1(s_grayPhotonVision, drivetrain, 0, 0.416, 0.17, () -> driver0.getRightY());
    // private PhotonCommand1 c_positionToLeftPole1 = new PhotonCommand1(s_bluePhotonVision, drivetrain, 0, 0.416, -0.17, () -> driver0.getRightY());

    
    public RobotContainer() {
  
    }
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

  
}