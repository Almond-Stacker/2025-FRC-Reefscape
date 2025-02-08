package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PrimaryElevatorCommand;
import frc.robot.Constants.PrimaryElevatorConsts;
import frc.robot.States.PrimaryElevatorStates;

public class PrimaryElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;
    private final DutyCycleEncoder absoluteEncoder;
    private final ProfiledPIDController elevatorPID;

    private boolean inBounds;
    private double motorOutput;

    private PrimaryElevatorCommand commands;

    public PrimaryElevatorSubsystem() {
        //motor configurations? and initializations
        leftElevatorMotor = new TalonFX(PrimaryElevatorConsts.leftElevatorMotorID);
        rightElevatorMotor = new TalonFX(PrimaryElevatorConsts.rightElevatorMotorID);
        absoluteEncoder = new DutyCycleEncoder(PrimaryElevatorConsts.encoderID);

        elevatorPID = new ProfiledPIDController(PrimaryElevatorConsts.kP, PrimaryElevatorConsts.kI, PrimaryElevatorConsts.kD, PrimaryElevatorConsts.PROFILE);
        
        setHeight(getHeight());
        elevatorPID.setTolerance(PrimaryElevatorConsts.PID_TOLERANCE);
        motorOutput = 0;

        commands = new PrimaryElevatorCommand(this);
    }
    
    @Override
    public void periodic() {
        double relativeElevatorPosition = getHeight();

        if(relativeElevatorPosition >= PrimaryElevatorStates.MAX.height 
                || relativeElevatorPosition <= PrimaryElevatorStates.MIN.height) {
            leftElevatorMotor.set(0);
            rightElevatorMotor.set(0);
            inBounds = false;
        } else {
            inBounds = true;
            //goal points are set in command
            motorOutput = elevatorPID.calculate(getHeight());
            leftElevatorMotor.set(motorOutput);
            rightElevatorMotor.set(motorOutput);
        }

        setSmartdashboard();
    }

    public void setHeight(double height) {
        elevatorPID.setGoal(height);
    }

    public void reset() {
        elevatorPID.setGoal(getHeight()); //stops elevator from rotating
        elevatorPID.reset(getHeight()); //sets the error back to zero, including mainly the integral term
    }

    public boolean atHeight() {
        return elevatorPID.atGoal();
    }
    
    //either return changing variable or calculate here idk.
    public double getHeight() {
        return rightElevatorMotor.getPosition().getValueAsDouble() + 0.5;
    }

    private void setSmartdashboard() {
        //state is printed in Command conjugate
        SmartDashboard.putNumber("Primary elevator goal position", elevatorPID.getGoal().position);
        SmartDashboard.putBoolean("Primary elevator in bounds", inBounds);
        SmartDashboard.putNumber("Primary elevator speed", motorOutput);
        SmartDashboard.putNumber("Primary elevator position ", getHeight());
    }

    public PrimaryElevatorCommand getCommands() {
        return commands;
    }

}
