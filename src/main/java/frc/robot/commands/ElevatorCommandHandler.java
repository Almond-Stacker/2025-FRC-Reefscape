package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private SequentialCommandGroup command; 

    public ElevatorCommandHandler(InnerElevatorSubsystem innerElevatorSubsystem, PrimaryElevatorSubsystem primaryElevatorSubsystem, IntakeArmSubsystem intakeArmSubsystem) {
        this.innerElevatorSubsystem = innerElevatorSubsystem;
        this.primaryElevatorSubsystem = primaryElevatorSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;
    }

    public SequentialCommandGroup setElevators(ElevatorStates elevatorStates) {
        if(state.innerHeight < ElevatorStates.CRITICAL_POINT.innerHeight && 
                state.armAngle > ElevatorStates.CRITICAL_POINT.armAngle &&
                elevatorStates.armAngle < ElevatorStates.CRITICAL_POINT.armAngle) {

        } else {
            command = new SequentialCommandGroup(new InstantCommand(() -> setElevatorStates(elevatorStates)));
        }
        this.state = elevatorStates;
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

    private SequentialCommandGroup criticalPointCommand() {

    }
}