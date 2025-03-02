package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConsts;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmCommand {
    private final IntakeArmSubsystem arm;
    private Command command; 

    public IntakeArmCommand(IntakeArmSubsystem arm) {
        this.arm = arm;
    }

    public Command setArm(double angle) {
        SmartDashboard.putNumber("goal angle", angle);
        command = arm.runOnce(() -> arm.setAngle(angle))
                .until(arm::atAngle)
                .handleInterrupt(arm::resetIntakeArm);
        return command;
    }

    public Command setIntakeState(IntakeStates state) {
        SmartDashboard.putString(" intake state", state.toString());
        command = arm.run(() -> arm.setIntakeStates(state.speed)); //set timeout??

        return command;
    }

    public IntakeArmSubsystem getIntakeArmSubsystem() {
        return arm;
    }
}