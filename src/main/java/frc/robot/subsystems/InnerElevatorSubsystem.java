package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants;
import frc.robot.States.InnerElevatorStates;

public class InnerElevatorSubsystem extends SubsystemBase {
    private final SparkFlex elevatorMotor;
    private final PIDController elevatorPID;
    private final ElevatorFeedforward elevatorFeedforward;
    private final RelativeEncoder elevatorEncoder;
    private InnerElevatorStates state;
    private double innerElevatorPosition; 
    private double motorSpeed;
    private boolean inBounds;
    private boolean override;

    public InnerElevatorSubsystem() {
        this.state = InnerElevatorStates.HOME;
        elevatorMotor = new SparkFlex(Constants.InnerElevator.ElevatorMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
        //elevatorEncoder = elevatorMotor.getAbsoluteEncoder();
        elevatorFeedforward = new ElevatorFeedforward(Constants.InnerElevator.kS, Constants.InnerElevator.kG, Constants.InnerElevator.kV);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorPID = new PIDController(Constants.InnerElevator.kP, Constants.InnerElevator.kI, Constants.InnerElevator.kD);
        setInnerElevatorState(state);
        override = false;
    }

    @Override
    public void periodic() {
        innerElevatorPosition = elevatorEncoder.getPosition() + 0.01;
        inBounds = false;


        // positive speed goes up 
        if(override) {
            elevatorMotor.set(motorSpeed);
            inBounds = true;
        }
        else if(innerElevatorPosition >= InnerElevatorStates.MAX.height || innerElevatorPosition <= InnerElevatorStates.MIN.height) {
            elevatorMotor.set(0.0);
            inBounds = false;
        } else {
            motorSpeed = elevatorPID.calculate(innerElevatorPosition) + elevatorFeedforward.calculate(innerElevatorPosition);
            if(motorSpeed < -0.1){
                motorSpeed *= 0.2;
            }

            elevatorMotor.set(motorSpeed);
            //elevatorMotor.set(0);
            inBounds = true; 
        }
        setSmartdashboard();
    }

    public void setInnerElevatorState(InnerElevatorStates state) {
        this.elevatorPID.setSetpoint(state.height);
        this.state = state;
    }

    public void setInnerElevatorSpeed(double speed) {
        elevatorMotor.set(speed);
    }

    private void setSmartdashboard() {
        SmartDashboard.putString("Inner elevator state", state.toString());
        SmartDashboard.putBoolean("Inner elevator in bounds", inBounds);
        SmartDashboard.putNumber("Inner elevator speed", motorSpeed);
        SmartDashboard.putNumber("Inner elevator posotion ", innerElevatorPosition);
        SmartDashboard.putNumber("Inner elevator goal position", state.height);
    }
}