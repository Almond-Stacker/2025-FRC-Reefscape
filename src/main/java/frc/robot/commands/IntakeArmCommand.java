package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.IntakeArmStates;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmCommand {
    private final IntakeArmSubsystem intakeArm;
    private IntakeArmStates intakeArmState;
    private IntakeStates intakeState;
    private Command command;

    public IntakeArmCommand(IntakeArmSubsystem intakeArm) {
        this.intakeArm = intakeArm;
    }

    public Command setAngle(IntakeArmStates state) {
        command = intakeArm.run(() -> intakeArm.setArmAngle(state.angle));
        return command;
    }

    public Command setIntakeSpeed(IntakeStates state) {
        command = intakeArm.run(() -> intakeArm.setIntakeSpeed(state.speed));
        return command;
    }

    public IntakeArmStates getIntakeArmState() {
        return intakeArmState;
    }

    public IntakeStates getIntakeState() {
        return intakeState;
    }

    public IntakeArmSubsystem getIntakeArmSubsystem() {
        return intakeArm;
    }
}