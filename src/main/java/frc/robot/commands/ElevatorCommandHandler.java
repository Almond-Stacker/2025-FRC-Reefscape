package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

import frc.lib.util.Utilities;

public class ElevatorCommandHandler {
    private final InnerElevatorSubsystem innerElevatorSubsystem;
    private final PrimaryElevatorSubsystem primaryElevatorSubsystem;
    private final IntakeArmSubsystem intakeArmSubsystem;

    private ElevatorStates state;
    private Command command; 

    public ElevatorCommandHandler(InnerElevatorSubsystem innerElevatorSubsystem, PrimaryElevatorSubsystem primaryElevatorSubsystem, IntakeArmSubsystem intakeArmSubsystem) {
        this.innerElevatorSubsystem = innerElevatorSubsystem;
        this.primaryElevatorSubsystem = primaryElevatorSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;
    }

    public Command setElevators(ElevatorStates elevatorStates) {
        this.state = elevatorStates;
        command = new InstantCommand(() -> setElevatorStates(elevatorStates));
        return command;
    }

    public ElevatorStates getCurrentState() {
        return state;
    }

    private void setElevatorStates(ElevatorStates state) {
        innerElevatorSubsystem.setInnerElevatorState(state);
        primaryElevatorSubsystem.setElevatorState(state);
        intakeArmSubsystem.setArmState(state);
    }
}