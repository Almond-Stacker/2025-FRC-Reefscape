package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.InnerElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;

public class InnerElevatorCommand {
    private final InnerElevatorSubsystem innerElevator;
    private InnerElevatorStates state;
    private Command command;

    public InnerElevatorCommand(InnerElevatorSubsystem elevatorInner) {
        this.innerElevator = elevatorInner;
    }

    public Command set(InnerElevatorStates state) {
        this.state = state;
        command = innerElevator.run(() -> innerElevator.setHeight(state.height));
        return command;
    }

    public InnerElevatorStates getState() {
        return state;
    }

    public InnerElevatorSubsystem getInnerElevatorSubsystem() {
        return innerElevator;
    }
}