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

    public ElevatorCommandHandler(InnerElevatorSubsystem innerElevatorSubsystem, 
                        PrimaryElevatorSubsystem primaryElevatorSubsystem, 
                        IntakeArmSubsystem intakeArmSubsystem) {
        this.innerElevatorSubsystem = innerElevatorSubsystem;
        this.primaryElevatorSubsystem = primaryElevatorSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;

        state = ElevatorStates.STARTING_POSITION;
        setElevatorStates(state, true);
    }

    public Command setElevators(ElevatorStates elevatorStates, boolean isABS) {
        this.state = elevatorStates;
        command = new InstantCommand(() -> setElevatorStates(elevatorStates, isABS));
        return command;
    }

    public ElevatorStates getCurrentState() {
        return state;
    }

    private void setElevatorStates(ElevatorStates elevatorStates, boolean isABS) {
        SmartDashboard.putString("Elevator State", elevatorStates.toString());

        if(isABS) {
            innerElevatorSubsystem.setInnerElevatorHeight(elevatorStates.innerHeight);
            primaryElevatorSubsystem.setPrimaryElevatorHeight(elevatorStates.primaryHeight);
        } else {
            double curInnerRel = innerElevatorSubsystem.getRelativeHeight();
            double curPrimaryRel = primaryElevatorSubsystem.getRelativeHeight();

            //return [innerHeight, primaryHeight]
            double[] distributedHeights = 
                Utilities.distributeElevatorHeights(elevatorStates.totalRelativeHeight,
                                            curInnerRel, curPrimaryRel);

            SmartDashboard.putNumber("Inner Relative Goal", distributedHeights[0]);
            SmartDashboard.putNumber("Primary Relative Goal", distributedHeights[1]);
            innerElevatorSubsystem.setInnerElevatorHeight(distributedHeights[0], false);
            primaryElevatorSubsystem.setPrimaryElevatorHeight(distributedHeights[1], false);
        }

        intakeArmSubsystem.setArmPosition(elevatorStates.armAngle);
        
    }
}
