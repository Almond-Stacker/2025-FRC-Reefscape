package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.States.ArmStates;
import frc.robot.States.InnerElevatorStates;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevatorSubsystem;

// public class sigma extends Command {
//     private final CommandSwerveDrivetrain s;
//     private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
//             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//     int counter = 0; 

//     public sigma(CommandSwerveDrivetrain s) {
//         this.s =s;
//     }

//     @Override
//     public void initialize() {
//         s.applyRequest(() -> robotCentricDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0.3));
//     }

//     @Override
//     public void execute() {
//         s.applyRequest(() -> robotCentricDrive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0.3));
//         counter += 1;
//         SmartDashboard.putNumber("stnhaeou", counter);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

// }

public class sigma extends Command {
    private final CommandSwerveDrivetrain s;
        private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private int counter = 0;

    public sigma(CommandSwerveDrivetrain s) {
        this.s = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing sigma command");
        s.applyRequest(() -> robotCentricDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0.3));
    }

    @Override
    public void execute() {
        System.out.println("Executing sigma command");
        RobotContainer.drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(-1).withVelocityY(0).withRotationalRate(0)).execute();
        counter += 1;
        SmartDashboard.putNumber("stnhaeou", counter);
    }

    @Override
    public boolean isFinished() {
        if(counter == 10) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending sigma command");
        s.applyRequest(() -> robotCentricDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
