package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;


public class ElevatorCommandHandler {
    PrimaryElevatorCommand primaryCommand;
    InnerElevatorCommand innerCommand;
    IntakeArmCommand armCommand;

    PrimaryElevatorSubsystem primarySubsystem;
    InnerElevatorSubsystem innerSubsystem;
    IntakeArmSubsystem armSubsystem;

    //rename 2/26
    public ElevatorCommandHandler(PrimaryElevatorSubsystem elevatorPrimary, InnerElevatorSubsystem elevatorInner, IntakeArmSubsystem arm) {
        primarySubsystem = elevatorPrimary;
        innerSubsystem = elevatorInner;
        armSubsystem = arm;

        primaryCommand = elevatorPrimary.getCommands();
        innerCommand = elevatorInner.getCommands();
        armCommand = arm.getCommands();

        setABS(ElevatorStates.HOME_ABS);
    }

    public void setABS(ElevatorStates state) {
        SmartDashboard.putString("Elevator state", state.toString());

        primaryCommand.set(state.primaryHeight).initialize();
        innerCommand.set(state.innerHeight).initialize();
        armCommand.setArm(state.angle).initialize();
    }
    //finds minimum distance for both elevators to move to total height give current height of both
    //total Relative Goal Height bounds {x|0<=x<=2}
    public void setElevatorHeight(double totalRelativeGoalHeight) {
        double relInnerHeight = getRelativeInnerHeight();
        double relPrimaryHeight = getRelativePrimaryHeight();
        double relDistributedGoalHeight = (totalRelativeGoalHeight - (relInnerHeight + relPrimaryHeight))/2;

        //ik ghetto but whatever
        double new_relInnerHeight = Math.max(0, Math.min(1, relInnerHeight + relDistributedGoalHeight));
        double new_relPrimaryHeight = Math.max(0, Math.min(1, relPrimaryHeight + 2*relDistributedGoalHeight + relInnerHeight - new_relInnerHeight));
        double differencedHeight = new_relInnerHeight + new_relPrimaryHeight - totalRelativeGoalHeight;
        if(differencedHeight >= 0.1) {
            new_relInnerHeight = Math.max(0, Math.min(1, new_relInnerHeight + differencedHeight));
        }

        innerCommand.set(relToAbsInnerHeight(new_relInnerHeight)).initialize();
        primaryCommand.set(relToAbsPrimaryHeight(new_relPrimaryHeight)).initialize();

    }

    //max total relative Height is 2
    public double getRelativeInnerHeight() {
        double height = innerSubsystem.getHeight();
        return (height - ElevatorStates.MIN_ABS.innerHeight)/(ElevatorStates.MAX_ABS.innerHeight - ElevatorStates.MIN_ABS.innerHeight);
    }

    public double relToAbsInnerHeight(double relInnerHeight) {
        return (ElevatorStates.MAX_ABS.innerHeight - ElevatorStates.MIN_ABS.innerHeight) * relInnerHeight + ElevatorStates.MIN_ABS.innerHeight;
    }

    public double getRelativePrimaryHeight() {
        double height = primarySubsystem.getHeight();
        return (height - ElevatorStates.MIN_ABS.primaryHeight)/(ElevatorStates.MAX_ABS.primaryHeight - ElevatorStates.MIN_ABS.primaryHeight);
    }

    public double relToAbsPrimaryHeight(double relPrimaryHeight) {
        return (ElevatorStates.MAX_ABS.primaryHeight - ElevatorStates.MIN_ABS.primaryHeight) * relPrimaryHeight + ElevatorStates.MIN_ABS.primaryHeight;
    }
}