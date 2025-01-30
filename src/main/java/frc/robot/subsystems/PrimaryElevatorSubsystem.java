package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.States.PrimaryElevatorStates;

public class PrimaryElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;
    private final DutyCycleEncoder encoder; 
    private final PIDController elevatorPID;

    private PrimaryElevatorStates state;
    private double elevatorPosition; 
    private double motorSpeed;
    private boolean inBounds;

    public PrimaryElevatorSubsystem() {
        this.state = PrimaryElevatorStates.HOME;
        leftElevatorMotor = new TalonFX(Constants.PrimaryElevator.leftElevatorMotorID);
        rightElevatorMotor = new TalonFX(Constants.PrimaryElevator.rightElevatorMotorID);  
        encoder = new DutyCycleEncoder(Constants.PrimaryElevator.encoderID);
        elevatorPID = new PIDController(Constants.PrimaryElevator.kP, Constants.PrimaryElevator.kI, Constants.PrimaryElevator.kD);
        setElevatorState(state);

        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
       // elevatorPosition = Units.degreesToRadians(leftElevatorMotor.getPosition().getValueAsDouble());
       elevatorPosition = Units.rotationsToDegrees(encoder.get());
        
        if(elevatorPosition >= PrimaryElevatorStates.MAX.height || elevatorPosition <= PrimaryElevatorStates.MIN.height) {
            leftElevatorMotor.set(0);
            rightElevatorMotor.set(0);
            inBounds = false;
        } else {
            motorSpeed = elevatorPID.calculate(elevatorPosition);//leftElevatorMotor.getPosition().getValueAsDouble());
            leftElevatorMotor.set(0);
            rightElevatorMotor.set(0);
            inBounds = true;    
        }
        setSmartdashboard();
    }

    public void setElevatorState(PrimaryElevatorStates state) {
        this.elevatorPID.setSetpoint(state.height);
        this.state = state;
    }

    private void setSmartdashboard() {
        SmartDashboard.putString("Primary elevator state", state.toString());
        SmartDashboard.putBoolean("Primary elevator in bounds", inBounds);
        SmartDashboard.putNumber("Primary elevator speed", motorSpeed);
        SmartDashboard.putNumber("Primary elevator position ", elevatorPosition);
    }
}