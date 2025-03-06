package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class CriticalPointCommand extends Command {
    private final IntakeArmSubsystem intakeArm;
    private final InnerElevatorSubsystem elevator;
    private final PrimaryElevatorSubsystem primaryElevator;
    private final ElevatorStates desiredElevatorStates;
    

    private boolean reachedIntakeArm;

    public CriticalPointCommand(IntakeArmSubsystem i, InnerElevatorSubsystem e, PrimaryElevatorSubsystem ee, ElevatorStates desiredStates) {
        this.intakeArm = i; 
        this.elevator = e; 
        this.desiredElevatorStates = desiredStates;
        this.primaryElevator = ee;
        this.reachedIntakeArm = false;
    }

    @Override
    public void initialize()  {
        
    }

    @Override 
    public void execute() {
        elevator.setInnerElevatorState(ElevatorStates.PRE_INTAKE);
        if(Math.abs(elevator.distanceFromSetpoint()) < 0.5) {
            intakeArm.setArmState(desiredElevatorStates);
            reachedIntakeArm = true;
        }
    }

    @Override
    public boolean isFinished() {
        if(reachedIntakeArm) {
            return true;
        }
        return false;
    }



    
}
