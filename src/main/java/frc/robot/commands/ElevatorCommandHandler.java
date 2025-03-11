package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

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
        state = ElevatorStates.STARTING_POSITION;
        setElevatorStates(ElevatorStates.STARTING_POSITION);
    }

    public SequentialCommandGroup setElevators(ElevatorStates elevatorStates) {
        command = new SequentialCommandGroup(new InstantCommand(() -> setElevatorStates(elevatorStates)));

        if((state.equals(ElevatorStates.L1) || state.equals(ElevatorStates.L2)) && !(elevatorStates.equals(ElevatorStates.L1) && elevatorStates.equals(ElevatorStates.L2))) {
            command = new SequentialCommandGroup(new InstantCommand (() -> innerElevatorSubsystem.setInnerElevatorState(elevatorStates.innerHeight)),
                new InstantCommand(() -> primaryElevatorSubsystem.setElevatorState(elevatorStates)), 
                new WaitCommand(0.3),
                new InstantCommand(() -> intakeArmSubsystem.setArm(elevatorStates.armAngle)));    
        }

        if(!(state.equals(ElevatorStates.L1) && state.equals(ElevatorStates.L2)) && (elevatorStates.equals(ElevatorStates.L1) || elevatorStates.equals(ElevatorStates.L2))) {
            command = new SequentialCommandGroup(new InstantCommand (() -> intakeArmSubsystem.setArm(elevatorStates.armAngle)),
                new InstantCommand(() -> primaryElevatorSubsystem.setElevatorState(elevatorStates)), 
                new WaitCommand(0.3),
                new InstantCommand(() -> innerElevatorSubsystem.setInnerElevatorState(elevatorStates.innerHeight)));
        }

        this.state = elevatorStates;
        return command;

    }

    public ElevatorStates getCurrentState() {
        return state;
    }

    private void setElevatorStates(ElevatorStates state) {
        innerElevatorSubsystem.setInnerElevatorState(state.innerHeight);
        primaryElevatorSubsystem.setElevatorState(state);
        intakeArmSubsystem.setArm(state.armAngle);
    }
}