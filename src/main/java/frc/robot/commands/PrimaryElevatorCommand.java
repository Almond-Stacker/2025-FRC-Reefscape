package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class PrimaryElevatorCommand {
    private final PrimaryElevatorSubsystem elevatorPrimary;
    private Command command;
    private PrimaryElevatorStates state;

    public PrimaryElevatorCommand(PrimaryElevatorSubsystem elevatorPrimary) {
        this.elevatorPrimary = elevatorPrimary;
    }

    public Command set(PrimaryElevatorStates state) {
        this.state = state;
        command = elevatorPrimary.runOnce(() -> elevatorPrimary.setHeight(state.height));
        return command;
    }

    public PrimaryElevatorStates getState() {
        return state;
    }

    public PrimaryElevatorSubsystem getPrimaryElevator() {
        return elevatorPrimary;
    }
}