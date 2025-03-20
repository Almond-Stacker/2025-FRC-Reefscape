package frc.robot.commands.teleopCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Utilities;
import frc.robot.Constants.ControllerConsts;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.InnerElevator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;


public class basicTeleop extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1);
    private final InnerElevator innerElevator; 

    private double xSpeed, ySpeed, rSpeed;

    public basicTeleop(CommandXboxController controller, CommandSwerveDrivetrain drivetrain, InnerElevator innerElevator) {
        this.controller = controller;
        this.drivetrain = drivetrain;
        this.innerElevator = innerElevator; 

        addRequirements(drivetrain);
    }

    @Override 
    public void execute() {
        xSpeed = controller.getLeftY();
        ySpeed = controller.getLeftX();
        rSpeed = controller.getRightX();

        if(!innerElevator.getInnerElevatorState().equals(ElevatorStates.STARTING_POSITION)) {
            xSpeed *= 0.4;
            ySpeed *= 0.4;
            rSpeed *= 0.4;
        } 

        drivetrain.applyRequest(() -> drive.withVelocityX(-xSpeed)
            .withVelocityY(-ySpeed)
            .withRotationalRate(-rSpeed)).execute();
    }
}