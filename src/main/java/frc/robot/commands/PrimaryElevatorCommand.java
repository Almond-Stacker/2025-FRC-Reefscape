package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class PrimaryElevatorCommand {
    private final PrimaryElevatorSubsystem elevatorPrimary;
    private Command command;

    public PrimaryElevatorCommand(PrimaryElevatorSubsystem elevatorPrimary) {
        this.elevatorPrimary = elevatorPrimary;
        this.elevatorPrimary.reset();
    }

    public Command set(double primaryHeight) {
        SmartDashboard.putNumber("goal Primary Height", primaryHeight);
        command = elevatorPrimary
            .runOnce(() -> elevatorPrimary.setHeight(primaryHeight))
            .until(elevatorPrimary::atHeight)
            .handleInterrupt(elevatorPrimary::reset);
        

        return command;
    }

    public PrimaryElevatorSubsystem getPrimaryElevator() {
        return elevatorPrimary;
    }

}

