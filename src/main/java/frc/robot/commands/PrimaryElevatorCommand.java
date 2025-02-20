package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class PrimaryElevatorCommand {
    
    private PrimaryElevatorSubsystem elevatorPrimary;

    public PrimaryElevatorCommand(PrimaryElevatorSubsystem elevatorPrimary) {
        this.elevatorPrimary = elevatorPrimary;
    }

    public Command set(PrimaryElevatorStates state) {
        SmartDashboard.putString("Primary elevator state", state.toString());
        return elevatorPrimary
            .runOnce(() -> elevatorPrimary.setHeight(state.height))
            .until(elevatorPrimary::atHeight)
            .handleInterrupt(elevatorPrimary::reset);
    }

    public PrimaryElevatorSubsystem getPrimaryElevator() {
        return elevatorPrimary;
    }

}