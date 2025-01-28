package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.States.ArmStates;
import frc.robot.States.IndexStates;
import frc.robot.States.InnerElevatorStates;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class PrimaryElevatorCommand extends Command {
    private final PrimaryElevatorSubsystem elevatorSubsystem;
    private final PrimaryElevatorStates elevatorState;

    public PrimaryElevatorCommand(PrimaryElevatorSubsystem elevatorSubsystem, PrimaryElevatorCommand config) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorState = config.elevatorState;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        if(elevatorState != null){
            elevatorSubsystem.setElevatorState(elevatorState);;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    

    public static class PrimaryElevatorCommandConfiguration {
        private InnerElevatorStates elevatorState;

        public PrimaryElevatorCommandConfiguration() {
            this.elevatorState = null;
        }

        public PrimaryElevatorCommandConfiguration withPrimaryElevator(InnerElevatorStates state) {
            this.elevatorState = state;
            return this;
        }

        public PrimaryElevatorCommandConfiguration build() {
            return this;
        }
    }
}