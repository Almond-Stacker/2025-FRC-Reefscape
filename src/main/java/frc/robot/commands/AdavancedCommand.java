package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.States.InnerElevatorStates;
import frc.robot.States.IntakeArmStates;
import frc.robot.States.IntakeStates;
import frc.robot.States.PrimaryElevatorStates;

public class AdavancedCommand {
    private final ClimbCommand climb;
    private final InnerElevatorCommand innerElevator;
    private final PrimaryElevatorCommand primaryElevator;
    private final IntakeArmCommand intakeArm; 

    public AdavancedCommand(ClimbCommand climbCommand, InnerElevatorCommand innerElevatorCommand, PrimaryElevatorCommand primaryElevatorCommand, IntakeArmCommand intakeArmCommand) {
        this.climb = climbCommand;
        this.innerElevator = innerElevatorCommand;
        this.primaryElevator = primaryElevatorCommand;
        this.intakeArm = intakeArmCommand;
    }

    public SequentialCommandGroup home() {
        return new SequentialCommandGroup(
            primaryElevator.set(PrimaryElevatorStates.STARTING_POSITION),
            innerElevator.set(InnerElevatorStates.STARTING_POSITION),
            intakeArm.setAngle(IntakeArmStates.STARTING_POSITION),
            intakeArm.setIntakeSpeed(IntakeStates.STOP)
    );
    }

    public SequentialCommandGroup stopALL() {

    }
}
