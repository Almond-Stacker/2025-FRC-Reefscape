package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.commands.PrimaryElevatorCommand;

public class PrimaryElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;
    private final PIDController elevatorPID;
    private final PrimaryElevatorCommand command;

    private double position; 
    private double motorSpeed;
    private boolean inBounds;

    public PrimaryElevatorSubsystem() {
        leftElevatorMotor = new TalonFX(Constants.PrimaryElevator.leftElevatorMotorID);
        rightElevatorMotor = new TalonFX(Constants.PrimaryElevator.rightElevatorMotorID);  
        elevatorPID = new PIDController(Constants.PrimaryElevator.kP, Constants.PrimaryElevator.kI, Constants.PrimaryElevator.kD);        
        command = new PrimaryElevatorCommand(this);
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
       position = rightElevatorMotor.getPosition().getValueAsDouble() + 0.5;
       inBounds = false;

       // positive goes up 
        if(position >= PrimaryElevatorStates.MAX.height || position <= PrimaryElevatorStates.MIN.height) {
            leftElevatorMotor.set(0);
            rightElevatorMotor.set(0);
            return;
        }
        
        motorSpeed = elevatorPID.calculate(position);
        leftElevatorMotor.set(motorSpeed);
        rightElevatorMotor.set(motorSpeed);
        inBounds = true;    
        setSmartdashboard();
    }

    public void setHeight(double height) {
        this.elevatorPID.setSetpoint(height);
    }

    private void setSmartdashboard() {
        SmartDashboard.putBoolean("Primary elevator in bounds", inBounds);
        SmartDashboard.putNumber("Primary elevator speed", motorSpeed);
        SmartDashboard.putNumber("Primary elevator current height", position);

        SmartDashboard.putNumber("Primary elevator goal height", command.getState().height);
        SmartDashboard.putString("Primary elevator state", command.getState().toString());
    }
}