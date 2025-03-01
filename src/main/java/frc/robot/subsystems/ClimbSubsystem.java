package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.ClimbCommand;

public class ClimbSubsystem extends SubsystemBase{
    private final TalonFX rightMotor;
    private final TalonFX leftMotor;
    private final ClimbCommand commands;
    private double speed = 0; 

    public ClimbSubsystem() {
        this.rightMotor = new TalonFX(20);
        this.rightMotor.setNeutralMode(NeutralModeValue.Brake);
        this.leftMotor = new TalonFX(21);
        this.leftMotor.setNeutralMode(NeutralModeValue.Brake);
        this.commands = new ClimbCommand(this);
    }

    @Override
    public void periodic() {
        rightMotor.set(speed);
        leftMotor.set(speed);
        setSmartdashboard();
    }

    public void setClimb(double speed) {
        this.speed = speed;
    }

    public void setSmartdashboard() {
        SmartDashboard.putNumber("Climb motor speed", commands.getState().speed);
    }

    public ClimbCommand getCommand() {
        return commands;
    }
}