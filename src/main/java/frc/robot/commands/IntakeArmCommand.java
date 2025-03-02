package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConsts;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmCommand {
    private final IntakeArmSubsystem arm;

    public IntakeArmCommand(IntakeArmSubsystem arm) {
        this.arm = arm;
    }

    public Command setArm(double angle) {
        SmartDashboard.putNumber("goal angle", angle);
        return arm
            .runOnce(() -> arm.setAngle(angle))
            .until(arm::atAngle)
            .handleInterrupt(arm::resetIntakeArm);
    }

    public Command setIntakeState(IntakeStates state) {
        SmartDashboard.putString(" intake state", state.toString());
        Command command = arm.run(() -> arm.setIntakeStates(state.speed)); //set timeout??

        if(state.equals(IntakeStates.FEED_OUT)) {
            return command
                .withTimeout(IntakeArmConsts.OUT_TIMEOUT)//seconds on
                .finallyDo(() -> arm.resetIntake());
        }

        return command.finallyDo(() -> arm.resetIntake());
    }

    public IntakeArmSubsystem getIntakeArmSubsystem() {
        return arm;
    }
}