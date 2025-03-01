package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.InnerElevatorSubsystem;

public class InnerElevatorCommand {
    private InnerElevatorSubsystem elevatorInner;

    public InnerElevatorCommand(InnerElevatorSubsystem elevatorInner) {
        this.elevatorInner = elevatorInner;
        this.elevatorInner.reset();
    }

    public Command set(double innerHeight) {
        SmartDashboard.putNumber("goal Inner Height", innerHeight);
        return elevatorInner
            .runOnce(() -> elevatorInner.setHeight(innerHeight))
            .until(elevatorInner::atHeight)
            .handleInterrupt(elevatorInner::reset);
    }

    public InnerElevatorSubsystem getInnerElevatorSubsystem() {
        return elevatorInner;
    }
}
