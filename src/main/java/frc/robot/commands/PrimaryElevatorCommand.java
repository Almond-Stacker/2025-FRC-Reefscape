package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.States.IntakeArmStates;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class PrimaryElevatorCommand {
    private final PrimaryElevatorSubsystem elevatorPrimary;
    private final IntakeArmSubsystem intakeArm;
    private Command command;
    private PrimaryElevatorStates state;

    public PrimaryElevatorCommand(PrimaryElevatorSubsystem elevatorPrimary, IntakeArmSubsystem intakeArm) {
        this.elevatorPrimary = elevatorPrimary;
        this.intakeArm = intakeArm;
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