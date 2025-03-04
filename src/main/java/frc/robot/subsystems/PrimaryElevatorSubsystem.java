package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PrimaryElevatorConsts;
import frc.robot.States.ElevatorStates;

public class PrimaryElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;
    private final PIDController elevatorPID;

    private boolean inBounds;
    private double motorOutput;
    private double currentHeight;
    private double goalPosition;

    public PrimaryElevatorSubsystem() {
        //motor configurations? and initializations
        leftElevatorMotor = new TalonFX(PrimaryElevatorConsts.leftElevatorMotorID);
        rightElevatorMotor = new TalonFX(PrimaryElevatorConsts.rightElevatorMotorID);
        elevatorPID = new PIDController(PrimaryElevatorConsts.kP, PrimaryElevatorConsts.kI, PrimaryElevatorConsts.kD);
        
        //config
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        disableSubsystem();
    }
    
    @Override
    public void periodic() {
        currentHeight = getHeight();
        inBounds = false;

        // move out of the min and max zones 
        if(currentHeight < ElevatorStates.MIN.primaryHeight) {
            motorOutput = -0.1;
        }
        else if(currentHeight > ElevatorStates.MAX.primaryHeight) {
            motorOutput = 0.1; 
        } else {
            motorOutput = elevatorPID.calculate(currentHeight);
            inBounds = true;
        }
        
        setMotorSpeed(motorOutput); 
        setSmartdashboard();
    }

    public void setPrimaryElevatorHeight(ElevatorStates primaryElevatorStates) {
        goalPosition = primaryElevatorStates.primaryHeight;
        elevatorPID.setSetpoint(goalPosition);
    }

    public double getHeight() {
        return rightElevatorMotor.getPosition().getValueAsDouble() + 0.5;
    }

    public double getRelativeHeight() {
        return (getHeight() - ElevatorStates.MIN.primaryHeight) / (ElevatorStates.MAX.primaryHeight - ElevatorStates.MIN.primaryHeight);
    }

    private void disableSubsystem() {
        leftElevatorMotor.disable();
        rightElevatorMotor.disable();
    }

    private void setMotorSpeed(double speed) {
        //leftElevatorMotor.set(speed);
        //rightElevatorMotor.set(speed);
    }

    private void setSmartdashboard() {
        SmartDashboard.putNumber("Primary Elevator Motor Output", motorOutput);
        SmartDashboard.putNumber("Primary Elevator Current Height", currentHeight);
        SmartDashboard.putNumber("Primary Elevator Relative Height", getRelativeHeight());
        SmartDashboard.putNumber("Primary Elevator Goal Height", goalPosition);

        SmartDashboard.putBoolean("Primary Elevator In Bounds", inBounds);
    }
}