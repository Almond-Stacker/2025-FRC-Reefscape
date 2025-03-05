package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.States.IntakeStates;

import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeCommand {
    private final IntakeArmSubsystem intake;
    private IntakeStates state;

    public IntakeCommand(IntakeArmSubsystem intake) {
        this.intake = intake;
        state = IntakeStates.STOP;
        intake.setIntakeSpeed(state.speed);
    } 

    public IntakeStates getCurrentState() {
        return state;
    }

    public Command setIntake(IntakeStates intakeState) {
        this.state = intakeState;
        return new InstantCommand(() -> intake.setIntakeSpeed(state.speed), intake);
    }
}
