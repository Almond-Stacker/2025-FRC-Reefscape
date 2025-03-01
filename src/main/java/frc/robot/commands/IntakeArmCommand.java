package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConsts;
import frc.robot.States.ElevatorStates;
import frc.robot.States.SuckStates;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmCommand {
    
    IntakeArmSubsystem arm;

    public IntakeArmCommand(IntakeArmSubsystem arm) {
        this.arm = arm;
        this.arm.resetArm();
    }

    public Command setArm(double angle) {
        SmartDashboard.putNumber("goal angle", angle);
        return arm
            .runOnce(() -> arm.setAngle(angle))
            .until(arm::atAngle)
            .handleInterrupt(arm::resetArm);
    }

    public Command setSuck(SuckStates state) {
        SmartDashboard.putString("Suck state", state.toString());
        Command command = arm.run(() -> arm.setSuckState(state)); //set timeout??

        if(state.equals(SuckStates.FEED_OUT)) {
            return command
                .withTimeout(IntakeArmConsts.OUT_TIMEOUT)//seconds on
                .finallyDo(() -> arm.setSuckState(SuckStates.STOP));
        }

        return command.finallyDo(() -> arm.setSuckState(SuckStates.STOP));
    }

    public IntakeArmSubsystem getIntakeArmSubsystem() {
        return arm;
    }
}