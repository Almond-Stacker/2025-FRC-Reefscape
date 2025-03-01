package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ArmStates;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmCommand {
    private final IntakeArmSubsystem intakeArm;
    private Command command;

    public IntakeArmCommand(IntakeArmSubsystem intakeArm) {
        this.intakeArm = intakeArm;
        setArmAngle(ArmStates.STARTING_POSITION);
        setIntakeSpeed(IntakeStates.STOP);
    }

    public Command setArmAngle(ArmStates state) {
        SmartDashboard.putString("Intake Arm State", state.toString());
        SmartDashboard.putNumber("Intake Arm Goal Angle", state.angle);
        
        command = intakeArm.run(() -> intakeArm.setArmAngle(state.angle));

        return command;
    }

    public Command setIntakeSpeed(IntakeStates state) {
        SmartDashboard.putString("Intake Arm State", state.toString());
        SmartDashboard.putNumber("Intake Arm Goal Angle", state.speed);
        
        command = intakeArm.run(() -> intakeArm.setArmAngle(state.speed));

        return command;
    }
}
