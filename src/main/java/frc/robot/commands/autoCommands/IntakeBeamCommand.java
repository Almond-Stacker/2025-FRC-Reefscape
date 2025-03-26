package frc.robot.commands.autoCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

//move back until beambreak is ran for more than
public class IntakeBeamCommand extends Command{

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric robotCentricDrive = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final BeamBreakSubsystem beamBreakSubsystem;
    private final double startTime;

    public IntakeBeamCommand(CommandSwerveDrivetrain drivetrain, BeamBreakSubsystem beamBreakSubsystem) {
        this.drivetrain = drivetrain;
        this.beamBreakSubsystem = beamBreakSubsystem;

        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void initialize() {
        // drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(-0.1)
        //     .withVelocityY(0)
        //     .withRotationalRate(0));
    }

    //might not need to but drive request in execute, instead initialize
    @Override
    public void execute() {
        //tune for speed
        drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(-0.1)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        //amt time for it to be recognized for it to be valid
        //so it can slide in lowkey, but also with timeout
        if(beamBreakSubsystem.getBrokenTime() > 0.5 || Timer.getFPGATimestamp() - startTime > 3) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean isFinished) {
        drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
}
