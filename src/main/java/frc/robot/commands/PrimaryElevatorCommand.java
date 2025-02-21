package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class PrimaryElevatorCommand {
    private final PrimaryElevatorSubsystem elevator;
    private Command command;

    public PrimaryElevatorCommand(PrimaryElevatorSubsystem elevator) {
        this.elevator = elevator;
        setElevator(PrimaryElevatorStates.HOME);
        
    }

    public Command setElevator(PrimaryElevatorStates state) {
        command = elevator.run(() -> elevator.setElevatorHeight(state.height));
        SmartDashboard.putString("Primary Elevator State", state.toString());
        SmartDashboard.putNumber("Primary Elevator Goal Height", state.height);
        return command;
    }
}