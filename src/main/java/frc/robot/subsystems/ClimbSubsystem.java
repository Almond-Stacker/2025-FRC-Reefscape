package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConsts;
import frc.robot.commands.ClimbCommand;

public class ClimbSubsystem extends SubsystemBase{
    private final TalonFX climbMotor;
    private final ClimbCommand commands;

    public ClimbSubsystem() {
        climbMotor = new TalonFX(ClimbConsts.climbMotorID);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);

        commands = new ClimbCommand(this);
    }
    
    public void setClimb(double speed) {
        climbMotor.set(speed);
    }

    public ClimbCommand getCommand() {
        return commands;
    }
}
