package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.InnerElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;

public class InnerElevatorCommand {
    private final InnerElevatorSubsystem elevator;
    private Command command;

    public InnerElevatorCommand(InnerElevatorSubsystem elevator) {
        this.elevator = elevator;
        setElevator(InnerElevatorStates.STARTING_POSITION);
    }

    public Command setElevator(InnerElevatorStates state) {
        command = elevator.run(() -> elevator.setElevatorHeight(state.height));
        SmartDashboard.putString("Inner Elevator State", state.toString());
        SmartDashboard.putNumber("Inner Elevator Goal Height", state.height);
        return command;
    }
}