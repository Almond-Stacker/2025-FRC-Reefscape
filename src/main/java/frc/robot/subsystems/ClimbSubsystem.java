package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConsts;
import frc.robot.States.ClimbStates;
import frc.robot.commands.ClimbCommand;


public class ClimbSubsystem extends SubsystemBase{
    private final TalonFX leftClimbMotor;
    
    private ClimbCommand commands; 

    public ClimbSubsystem() {
        leftClimbMotor = new TalonFX(ClimbConsts.leftClimbMotorID);
        leftClimbMotor.setNeutralMode(NeutralModeValue.Brake); 

        commands = new ClimbCommand(this); 
    }

    @Override
    public void periodic() {
        setSmartdashboard(); 
    }

    public void setClimb(double speed) {
        leftClimbMotor.set(speed); 
    }

    public void setSmartdashboard() {
        // idk what do put in here :()
    }

    public ClimbCommand getCommand () {
        return commands; 
    }    
}
