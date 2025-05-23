package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States.ElevatorStates;
import frc.robot.States.IndexStates;
import frc.robot.subsystems.InnerElevator;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.PrimaryElevator;

public class ElevatorCommandHandler {
    private final PrimaryElevator primaryElevatorSubsystem;
    private final InnerElevator innerElevatorSubsystem;
    private final IntakeArm armSubsystem;

    public ElevatorCommandHandler(PrimaryElevator primaryElevatorSubsystem, InnerElevator innerElevatorSubsystem, IntakeArm armSubsystem) {
        this.primaryElevatorSubsystem = primaryElevatorSubsystem;
        this.innerElevatorSubsystem = innerElevatorSubsystem;
        this.armSubsystem = armSubsystem;

        primaryElevatorSubsystem.setPrimaryElevatorState(ElevatorStates.STARTING_POSITION);
        innerElevatorSubsystem.setInnerElevatorState(ElevatorStates.STARTING_POSITION);
        armSubsystem.setArmState(ElevatorStates.STARTING_POSITION);
    }

    public InstantCommand setPrimaryElevatorState(ElevatorStates state) {
        return new InstantCommand(() -> primaryElevatorSubsystem.setPrimaryElevatorState(state), primaryElevatorSubsystem);
    }

    public InstantCommand setInnerElevatorState(ElevatorStates state) {
        return new InstantCommand(() -> innerElevatorSubsystem.setInnerElevatorState(state), innerElevatorSubsystem);
    }

    public InstantCommand setArmState(ElevatorStates state) {
        return new InstantCommand(() -> armSubsystem.setArmState(state), armSubsystem);
    }

    public SequentialCommandGroup setElevators(ElevatorStates state) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        if((innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.L1) || innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.L2))
            && (state.equals(ElevatorStates.PRE_INTAKE) || state.equals(ElevatorStates.INTAKE) || state.equals(ElevatorStates.STARTING_POSITION))) {
            command.addCommands(setPrimaryElevatorState(state), setInnerElevatorState(ElevatorStates.HIGH_PRE_INTAKE), new WaitCommand(0.3), 
                setArmState(state), new WaitCommand(0.4), setInnerElevatorState(state));
        } else if(innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.PRE_INTAKE) && state.equals(ElevatorStates.INTAKE)) {
            command.addCommands(setPrimaryElevatorState(state), setArmState(state), new WaitCommand(0.4), setInnerElevatorState(state));
        } else {
            command.addCommands(setPrimaryElevatorState(state), setInnerElevatorState(state), setArmState(state));
        }
        command.addCommands(new InstantCommand(() -> armSubsystem.setIndexState(IndexStates.STOP)));
        return command;
    }
    

    public ElevatorStates getState() {
        return innerElevatorSubsystem.getInnerElevatorState();
    }
}
//  