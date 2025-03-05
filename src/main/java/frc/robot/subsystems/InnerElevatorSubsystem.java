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
import frc.robot.Constants;
import frc.robot.Constants.InnerElevatorConsts;
import frc.robot.States.ElevatorStates;

public class InnerElevatorSubsystem extends SubsystemBase{
    private final SparkFlex elevatorMotor;
    private final PIDController elevatorPID;
    private final ElevatorFeedforward elevatorFeedForward;
    private final RelativeEncoder elevatorEncoder; 

    private double motorOutput;
    private boolean inBounds;
    private double currentHeight;
    private double goalPosition;
    private double addMotor = 0; //idc bout allocation arlen

    public InnerElevatorSubsystem() {   
        elevatorMotor = new SparkFlex(Constants.InnerElevatorConsts.elevatorMotorID, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorPID = new PIDController(InnerElevatorConsts.kP, InnerElevatorConsts.kI, InnerElevatorConsts.kD);
        elevatorFeedForward = new ElevatorFeedforward(InnerElevatorConsts.kS, InnerElevatorConsts.kG, InnerElevatorConsts.kV);

        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
        //disableSubsystem();

    }

    @Override
    public void periodic() {
        currentHeight = getHeight();
        inBounds = false;

        if(currentHeight > ElevatorStates.MAX.innerHeight) {
            motorOutput = -0.05;
        } else if(currentHeight < ElevatorStates.MIN.innerHeight) {
            motorOutput = 0.05;
        } else {
            motorOutput = elevatorPID.calculate(currentHeight)
                + elevatorFeedForward.calculate(elevatorPID.getSetpoint(), elevatorMotor.getEncoder().getVelocity());
            //elevatorPID.calculate(currentHeight) +
            // if (motorOutput < -0.1) {
            //     motorOutput *= 0.2;
            // }   
            inBounds = true;
        }
        
       // elevatorMotor.set(motorOutput + addMotor);
        setSmartdashboard();
    }

    public void setMotorSpeed(double speed) {
        addMotor = speed;
        //elevatorMotor.set(motorOutput);
    }

    public void setInnerElevatorHeight(double innerHeight) {
        goalPosition = innerHeight;
        elevatorPID.setSetpoint(goalPosition);
    }

    public void setInnerElevatorHeight(double innerHeight, boolean isABS) {
        if(isABS) {
            setInnerElevatorHeight(innerHeight);
        } else {
            setInnerElevatorHeight(relToABSHeight(innerHeight));
        }
    }

    public double getHeight() {
        return elevatorEncoder.getPosition() + 18;
    }

    public double getRelativeHeight() {
        return (getHeight() - ElevatorStates.MIN.innerHeight) / (ElevatorStates.MAX.innerHeight - ElevatorStates.MIN.innerHeight);
    }

    public double relToABSHeight(double relativeHeight) {
        return (ElevatorStates.MAX.innerHeight - ElevatorStates.MIN.innerHeight) * relativeHeight + ElevatorStates.MIN.innerHeight;
    }

    private void disableSubsystem() {
        elevatorMotor.disable();
    }

    private void setSmartdashboard() {
        SmartDashboard.putNumber("Inner Elevator Current Height", currentHeight);
        SmartDashboard.putNumber("Inner Elevator Goal Position", goalPosition);
        SmartDashboard.putNumber("Inner Elevator Relative Height", getRelativeHeight());
        SmartDashboard.putNumber("Inner Elevator Motor Output", motorOutput);
        
        SmartDashboard.putBoolean("Inner Elevator In Bounds", inBounds);
    }
}
