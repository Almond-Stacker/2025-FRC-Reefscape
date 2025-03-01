package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class PrimaryElevatorCommand {
    
    private PrimaryElevatorSubsystem elevatorPrimary;

    public PrimaryElevatorCommand(PrimaryElevatorSubsystem elevatorPrimary) {
        this.elevatorPrimary = elevatorPrimary;
        this.elevatorPrimary.reset();
    }

    public Command set(double primaryHeight) {
        SmartDashboard.putNumber("goal Primary Height", primaryHeight);
        return elevatorPrimary
            .runOnce(() -> elevatorPrimary.setHeight(primaryHeight))
            .until(elevatorPrimary::atHeight)
            .handleInterrupt(elevatorPrimary::reset);
    }

    public PrimaryElevatorSubsystem getPrimaryElevator() {
        return elevatorPrimary;
    }

}
