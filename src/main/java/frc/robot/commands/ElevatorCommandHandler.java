package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;


public class ElevatorCommandHandler extends Command {
    // PrimaryElevatorCommand primaryCommand;
    // InnerElevatorCommand innerCommand;
    // IntakeArmCommand armCommand;

    private final PrimaryElevatorSubsystem primarySubsystem;
    private final InnerElevatorSubsystem innerSubsystem;
    private final IntakeArmSubsystem armSubsystem;
    private final ElevatorStates state;

    private boolean isRestricted;
    private double targetPrimary, targetInner, targetArmAngle;
    private double safeArmAngle;

    //rename 2/26
    public ElevatorCommandHandler(PrimaryElevatorSubsystem elevatorPrimary, InnerElevatorSubsystem elevatorInner, IntakeArmSubsystem arm, ElevatorStates state) {
        primarySubsystem = elevatorPrimary;
        innerSubsystem = elevatorInner;
        armSubsystem = arm;
        addRequirements(primarySubsystem, innerSubsystem, armSubsystem);
        this.state = state;
    }

    @Override
    public void initialize() {
        double curInnerHeight = getRelativeInnerHeight();
        targetArmAngle = state.angle;
        isRestricted = isRestricted(curInnerHeight, targetArmAngle);
        safeArmAngle = getMinimumSafeArmAngle(curInnerHeight);

        if(state.isABS) {
            targetPrimary = state.primaryHeight;
            targetInner = state.innerHeight;
        } else {
            double totalRelHeight = state.relTotalHeight;
            SmartDashboard.putNumber("Relative Goal", totalRelHeight);
            double curPrimaryHeight = getRelativePrimaryHeight();

            double[] optimizedHeights = distributeElevatorHeights(totalRelHeight, curInnerHeight, curPrimaryHeight);
            targetInner = relToAbsInnerHeight(optimizedHeights[0]);
            targetPrimary = relToAbsPrimaryHeight(optimizedHeights[1]);
        }

        SmartDashboard.putString("Elevator state", state.toString());

        innerSubsystem.setHeight(targetInner);
        primarySubsystem.setHeight(targetPrimary);
    }

    @Override
    public void execute() {
        double curInnerHeight = getRelativeInnerHeight();
        if(isRestricted) {
            armSubsystem.setAngle(getMinimumSafeArmAngle(curInnerHeight));
        }
    }

    @Override
    public boolean isFinished() {
        boolean atTargetHeight = 
            Math.abs(innerSubsystem.getHeight() - targetInner) < 0.05 &&
            Math.abs(primarySubsystem.getHeight() - targetPrimary) < 0.05;
        if(atTargetHeight) {
            armSubsystem.setAngle(targetArmAngle);
        }

        return atTargetHeight;
    }  

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            armSubsystem.resetArm();
            innerSubsystem.reset();
            primarySubsystem.reset();
        }
    }

    private double[] distributeElevatorHeights(double relTotalHeight, double curInner, double curPrimary) {
        //SmartDashboard.putString("Elevator state", state.toString());
        //SmartDashboard.putNumber("Relative goal", state.relTotalHeight);

        double newRelInnerHeight = curInner + relTotalHeight / 2;
        double newRelPrimaryHeight = curPrimary + relTotalHeight / 2;

        if(newRelInnerHeight > 1) {
            double overflow = newRelInnerHeight - 1;
            newRelInnerHeight = 1;
            newRelPrimaryHeight = Math.min(newRelPrimaryHeight + overflow, 1);
        } else if(newRelInnerHeight < 0) { //in case something fricken messdup happened
            double deficit = newRelInnerHeight;
            newRelInnerHeight = 0;
            newRelPrimaryHeight = Math.max(newRelPrimaryHeight - deficit, 0);
        }

        if (newRelPrimaryHeight > 1) {
            double overflow = newRelPrimaryHeight - 1;
            newRelPrimaryHeight = 1;
            newRelInnerHeight = Math.min(newRelInnerHeight + overflow, 1);
        } else if (newRelPrimaryHeight < 0) { // angr nothing working, fail safe
            double deficit = newRelPrimaryHeight;
            newRelPrimaryHeight = 0;
            newRelInnerHeight = Math.max(newRelInnerHeight - deficit, 0);
        }
        
        return new double[]{newRelInnerHeight, newRelPrimaryHeight};
    }

    private boolean isRestricted(double innerHeight, double angle) {
        return (angle < 82) && innerHeight < 0.5;
    }

    

    private double getMinimumSafeArmAngle(double innerHeight) {
        double minSafeHeight = 0.5;
        if(innerHeight >= minSafeHeight) return 82;
        double heightDeficit = minSafeHeight - innerHeight;
        double safeAngle = Math.toDegrees(Math.atan(heightDeficit / 1));
        return Math.max(0, safeAngle);
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

