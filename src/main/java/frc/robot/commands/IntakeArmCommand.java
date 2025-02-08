package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.IntakeArmStates;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmCommand {
    
    IntakeArmSubsystem arm;

    public IntakeArmCommand(IntakeArmSubsystem arm) {
        this.arm = arm;
    }

    public Command set(IntakeArmStates state) {
        SmartDashboard.putString("Intake arm state", state.toString());
        return arm
            .runOnce(() -> arm.setAngle(state.angle))
            .until(arm::atAngle)
            .handleInterrupt(arm::reset);
    }
}
