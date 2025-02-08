package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.InnerElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;

public class InnerElevatorCommand {
    private InnerElevatorSubsystem elevatorInner;

    public InnerElevatorCommand(InnerElevatorSubsystem elevatorInner) {
        this.elevatorInner = elevatorInner;
    }

    public Command set(InnerElevatorStates state) {
        SmartDashboard.putString("Inner elevator state", state.toString());
        return elevatorInner
            .runOnce(() -> elevatorInner.setHeight(state.height))
            .until(elevatorInner::atHeight)
            .handleInterrupt(elevatorInner::reset);
    }

    public InnerElevatorSubsystem getInnerElevatorSubsystem() {
        return elevatorInner;
    }
}
