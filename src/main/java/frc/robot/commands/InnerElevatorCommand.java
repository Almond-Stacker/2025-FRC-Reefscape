package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.States.ArmStates;
import frc.robot.States.IndexStates;
import frc.robot.States.InnerElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.InnerElevatorSubsystem;

public class InnerElevatorCommand extends Command {
    private final InnerElevatorSubsystem elevatorSubsystem;
    private final InnerElevatorStates elevatorState;

    public InnerElevatorCommand(InnerElevatorSubsystem elevatorSubsystem, InnerElevatorCommandConfiguration config) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorState = config.elevatorState;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        if(elevatorState != null){
            elevatorSubsystem.setInnerElevatorState(elevatorState);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    

    public static class InnerElevatorCommandConfiguration {
        private InnerElevatorStates elevatorState;

        public InnerElevatorCommandConfiguration() {
            this.elevatorState = null;
        }

        public InnerElevatorCommandConfiguration withInnerElevator(InnerElevatorStates state) {
            this.elevatorState = state;
            return this;
        }

        public InnerElevatorCommandConfiguration build() {
            return this;
        }
    }
}
