package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

/* 
public class ElevatorCommand {
    private InnerElevatorSubsystem elevatorInner;
    private PrimaryElevatorSubsystem elevatorPrimary;

    public ElevatorCommand(PrimaryElevatorSubsystem elevatorPrimary, InnerElevatorSubsystem elevatorInner) {
        this.elevatorPrimary = elevatorPrimary;
        this.elevatorInner = elevatorInner;
    }

    public Command set(ElevatorStates state) {
        SmartDashboard.putString("Elevator state", state.toString());
        //sequential command group for settings heights on both primary and inner elevators
    }

    private double divideHeight(double height) {

    }
}
*/