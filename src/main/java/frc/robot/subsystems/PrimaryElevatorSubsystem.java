package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PrimaryElevatorCommand;
import frc.robot.Constants;
import frc.robot.Constants.PrimaryElevatorConsts;
import frc.robot.States.ElevatorStates;

public class PrimaryElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;
    private final PIDController elevatorPID;

    private boolean inBounds;
    private double motorOutput;
    private double currentHeight;

    private PrimaryElevatorCommand commands;

    public PrimaryElevatorSubsystem() {
        //motor configurations? and initializations
        leftElevatorMotor = new TalonFX(PrimaryElevatorConsts.leftElevatorMotorID);
        rightElevatorMotor = new TalonFX(PrimaryElevatorConsts.rightElevatorMotorID);
        elevatorPID = new PIDController(PrimaryElevatorConsts.kP, PrimaryElevatorConsts.kI, PrimaryElevatorConsts.kD);
        
        elevatorPID.setTolerance(PrimaryElevatorConsts.PID_TOLERANCE);
        motorOutput = 0;

        //config
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        commands = new PrimaryElevatorCommand(this);

        if(Constants.disableSubsystems) {
            disableSubsystem();
        }
    }
    
    @Override
    public void periodic() {
        currentHeight = getHeight();
        inBounds = false;

        // move out of the min and max zones 
        if(currentHeight > ElevatorStates.MAX_ABS.primaryHeight) {
            setMotorSpeed(-0.1);
            return;
        }

        if(currentHeight < ElevatorStates.MIN_ABS.primaryHeight) {
            setMotorSpeed(0.1);
            return; 
        }
        
        inBounds = true;
        //goal points are set in command
        motorOutput = elevatorPID.calculate(getHeight());
        //setMotorSpeed(motorOutput); 
        setSmartdashboard();
    }

    public void setHeight(double height) {
        elevatorPID.setSetpoint(height);
    }

    public void reset() {
        elevatorPID.setSetpoint(getHeight()); //stops elevator from moving
    }

    public boolean atHeight() {
        return elevatorPID.atSetpoint();
    }
    
    public double getHeight() {
        return rightElevatorMotor.getPosition().getValueAsDouble() + 0.5;
    }

    private void disableSubsystem() {
        leftElevatorMotor.disable();
        rightElevatorMotor.disable();
    }

    private void setMotorSpeed(double speed) {
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
    }

    private void setSmartdashboard() {
        //state is printed in Command conjugate
        SmartDashboard.putNumber("Primary elevator goal position", elevatorPID.getSetpoint());
        SmartDashboard.putBoolean("Primary elevator in bounds", inBounds);
        SmartDashboard.putNumber("Primary elevator speed", motorOutput);
        SmartDashboard.putNumber("Primary elevator position ", getHeight());
    }

    public PrimaryElevatorCommand getCommands() {
        return commands;
    }

}
