package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PrimaryElevatorConsts;
import frc.robot.States.ElevatorStates;

public class PrimaryElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;
    private final DutyCycleEncoder absoluteEncoder;
    private final PIDController elevatorPID;

    private boolean inBounds;
    private double motorOutput;

    public PrimaryElevatorSubsystem() {
        //motor configurations? and initializations
        leftElevatorMotor = new TalonFX(PrimaryElevatorConsts.leftElevatorMotorID);
        rightElevatorMotor = new TalonFX(PrimaryElevatorConsts.rightElevatorMotorID);
        absoluteEncoder = new DutyCycleEncoder(PrimaryElevatorConsts.encoderID);

        elevatorPID = new PIDController(PrimaryElevatorConsts.kP, PrimaryElevatorConsts.kI, PrimaryElevatorConsts.kD);
        
        setHeight(getHeight());
        elevatorPID.setTolerance(PrimaryElevatorConsts.PID_TOLERANCE);
        motorOutput = 0;

        //config
        absoluteEncoder.setDutyCycleRange(0, 1750);
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    @Override
    public void periodic() {
        double relativeElevatorPosition = getHeight();

        if(relativeElevatorPosition >= ElevatorStates.MAX_ABS.primaryHeight 
                || relativeElevatorPosition <= ElevatorStates.MIN_ABS.primaryHeight) {
            //leftElevatorMotor.set(0);
            //rightElevatorMotor.set(0);
            inBounds = false;
        } else {
            inBounds = true;
            //goal points are set in command
            motorOutput = elevatorPID.calculate(getHeight());
            //leftElevatorMotor.set(motorOutput);
            //rightElevatorMotor.set(motorOutput);
        }

        setSmartdashboard();
    }

    public void setHeight(double height) {
        elevatorPID.setSetpoint(height);
    }

    public void reset() {
        elevatorPID.setSetpoint(getHeight()); //stops elevator from rotating
    }

    public boolean atHeight() {
        return elevatorPID.atSetpoint();
    }

    public double relToAbsPrimaryHeight(double relPrimaryHeight) {
        return (ElevatorStates.MAX_ABS.primaryHeight - ElevatorStates.MIN_ABS.primaryHeight) * relPrimaryHeight + ElevatorStates.MIN_ABS.primaryHeight;
    }
    
    //either return changing variable or calculate here idk.
    public double getHeight() {
        return rightElevatorMotor.getPosition().getValueAsDouble() + 0.5;
    }

    public double getRelativePrimaryHeight() {
        return (getHeight() - ElevatorStates.MIN_ABS.primaryHeight)/(ElevatorStates.MAX_ABS.primaryHeight - ElevatorStates.MIN_ABS.primaryHeight);
    }

    private void setSmartdashboard() {
        //state is printed in Command conjugate
        SmartDashboard.putNumber("Primary elevator goal position", elevatorPID.getSetpoint());
        SmartDashboard.putBoolean("Primary elevator in bounds", inBounds);
        SmartDashboard.putNumber("Primary elevator speed", motorOutput);
        SmartDashboard.putNumber("Primary elevator position ", getHeight());
    }

}
