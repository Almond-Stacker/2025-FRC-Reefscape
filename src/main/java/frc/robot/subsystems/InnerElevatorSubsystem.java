package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants.InnerElevatorConsts;
import frc.robot.States.ElevatorStates;
import frc.robot.commands.InnerElevatorCommand;

public class InnerElevatorSubsystem extends SubsystemBase{
    private final SparkFlex motor;
    private final PIDController PID;
    private final ElevatorFeedforward feedFoward;
    private final RelativeEncoder encoder; 
    private final InnerElevatorCommand command;

    private double motorOutput;
    private boolean inBounds;

    public InnerElevatorSubsystem() {   
        motor = new SparkFlex(InnerElevatorConsts.elevatorMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(motor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);

        encoder = motor.getEncoder();

        PID = new PIDController(InnerElevatorConsts.kP, InnerElevatorConsts.kI, InnerElevatorConsts.kD);
        feedFoward = new ElevatorFeedforward(motorOutput, motorOutput, motorOutput);
        
        command = new InnerElevatorCommand(this);
    }

    @Override
    public void periodic() {
        double relativeElevatorPosition = getHeight();

        if(relativeElevatorPosition >= ElevatorStates.MAX_ABS.innerHeight
                || relativeElevatorPosition <= ElevatorStates.MIN_ABS.innerHeight) {
            motorOutput = 0;
            inBounds = false;
        } else {
            motorOutput = elevatorPID.calculate(getHeight()) + elevatorFeedforward.calculate(elevatorPID.getSetpoint(), elevatorMotor.getEncoder().getVelocity());
            inBounds = true;
        }
        //elevatorMotor.set(motorOutput);
        setSmartdashboard();
    }

    public void setHeight(double height) {
        elevatorPID.setSetpoint(height);
    }

    public void reset() {
        elevatorPID.setSetpoint(getHeight());
    }

    public boolean atHeight() {
        return elevatorPID.atSetpoint();
    }

    public double getHeight() {
        return elevatorEncoder.getPosition() + 0.01;
    }

    private void setSmartdashboard() {
        SmartDashboard.putBoolean("Inner elevator in bounds", inBounds);
        SmartDashboard.putNumber("Inner elevator speed", motorOutput);
        SmartDashboard.putNumber("Inner elevator posotion ", getHeight());
        SmartDashboard.putNumber("Inner elevator goal position", elevatorPID.getSetpoint());
    }

    public InnerElevatorCommand getCommands() {
        return command;
    }
}