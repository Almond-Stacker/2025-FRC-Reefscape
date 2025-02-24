package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ClimbStates;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand {
    private final ClimbSubsystem climb;
    private ClimbStates state;
    private Command command;

    public ClimbCommand(ClimbSubsystem climb) {
        this.climb = climb;
        setClimb(ClimbStates.STOP);
    }

    public Command setClimb(ClimbStates state) {
        this.state = state;
        command = climb.run(() -> climb.setClimb(state.speed));
        return command;
    }

    public ClimbStates getState() {
        return state;
    }

}