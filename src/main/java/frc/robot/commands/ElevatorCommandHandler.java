package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    
        set(ElevatorStates.HOME_ABS);
    }

    public void setABS(ElevatorStates state) {
        SmartDashboard.putString("Elevator state", state.toString());
        SmartDashboard.putNumber("Relative Goal", state.relTotalHeight);

        primaryCommand.set(state.primaryHeight).initialize();
        innerCommand.set(state.innerHeight).initialize();
        armCommand.setArm(state.angle).initialize();
    }
    //finds minimum distance for both elevators to move to total height give current height of both
    //total Relative Goal Height bounds {x|0<=x<=2}
    public double[] distributeElevatorHeights(ElevatorStates state) {
        SmartDashboard.putString("Elevator state", state.toString());
        SmartDashboard.putNumber("Relative Goal", state.relTotalHeight);
        
       double curInner = getRelativeInnerHeight();
        double curPrimary = getRelativePrimaryHeight();

        double availableInner = 1 - curInner;
        double availablePrimary = 1 - curPrimary;

        double remainingHeight = state.relTotalHeight - (curInner + curPrimary);
        double newRelInnerHeight = curInner + Math.min(remainingHeight / 2, availableInner);
        double newRelPrimaryHeight = curPrimary + Math.min(remainingHeight / 2, availablePrimary);

        if (newRelInnerHeight > 1) {
            double overflow = newRelInnerHeight - 1;
            newRelInnerHeight = 1;
            newRelPrimaryHeight = Math.min(newRelPrimaryHeight + overflow, 1);
        } else if (newRelPrimaryHeight > 1) {
            double overflow = newRelPrimaryHeight - 1;
            newRelPrimaryHeight = 1;
            newRelInnerHeight = Math.min(newRelInnerHeight + overflow, 1);
        }

        //lower bound
        newRelInnerHeight = Math.max(newRelInnerHeight, 0);
        newRelPrimaryHeight = Math.max(newRelPrimaryHeight, 0);

        SmartDashboard.putNumber("Relative Inner Height", newRelInnerHeight);
        SmartDashboard.putNumber("Relative Primary Height", newRelPrimaryHeight);

        return new double[]{newRelInnerHeight, newRelPrimaryHeight};
    }

    //can't run with command I think, no subsystem
    public SequentialCommandGroup set(ElevatorStates state) {
        if(!state.ABS) {
            double[] distribuedHeights = distributeElevatorHeights(state);
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                primaryCommand.set(relToAbsPrimaryHeight(distribuedHeights[1]));
                innerCommand.set(relToAbsInnerHeight(distribuedHeights[0]));
            }).onlyIf(() -> isRestricted(getRelativeInnerHeight(), state.angle)),
            new InstantCommand(() -> {
                primaryCommand.set(relToAbsPrimaryHeight(distribuedHeights[1]));
                innerCommand.set(relToAbsInnerHeight(distribuedHeights[0]));
                armCommand.setArm(state.angle);
            })
        );
        } else {
            return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    primaryCommand.set(state.primaryHeight);
                    innerCommand.set(state.innerHeight);
                }).onlyIf(() -> isRestricted(innerSubsystem.getHeight(), state.angle)),
                new InstantCommand(() -> {
                    primaryCommand.set(state.primaryHeight);
                    innerCommand.set(state.innerHeight);
                    armCommand.setArm(state.angle);
                })
            );
        }
        
    }

    //ABS
    public boolean isRestricted(double targetABSInnerHeight, double targetABSAngle) {
        if(targetABSInnerHeight < 0.5 && targetABSAngle < 82.9) {
            return true;
        } 
        return false;
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