package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.ClimbCommand;

public class ClimbSubsystem extends SubsystemBase{
    private final TalonFX climbMotor;
    private final ClimbCommand commands;

    public ClimbSubsystem() {
        this.climbMotor = new TalonFX(ClimbConstants.climbMotorID);
        this.climbMotor.setNeutralMode(NeutralModeValue.Brake);
        this.commands = new ClimbCommand(this);
    }

    @Override
    public void periodic() {
        setSmartdashboard();
    }

    public void setClimb(double speed) {
        climbMotor.set(speed);
    }

    public void setSmartdashboard() {

    }

    public ClimbCommand getCommand() {
        return commands;
    }
}