package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtil;
import frc.lib.util.Utilities;
import frc.lib.util.SparkFlexUtil.Usage;
import frc.robot.Constants;
import frc.robot.States.InnerElevatorStates;

public class InnerElevatorSubsystem extends SubsystemBase {
    private final SparkFlex elevatorMotor;
    private final PIDController elevatorPID;
   // private final ArmFeedforward armFeedforward;
    //private final SparkAbsoluteEncoder elevatorEncoder;
    private final RelativeEncoder elevatorEncoder;
    private InnerElevatorStates state;
    private double innerElevatorPosition; 
    private double motorSpeed;
    private boolean inBounds;

    public InnerElevatorSubsystem() {
        this.state = InnerElevatorStates.L1;
        elevatorMotor = new SparkFlex(Constants.InnerElevator.ElevatorMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
        //elevatorEncoder = elevatorMotor.getAbsoluteEncoder();
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorPID = new PIDController(Constants.InnerElevator.kP, Constants.InnerElevator.kI, Constants.InnerElevator.kD);
        setInnerElevatorState(state);
    }

    @Override
    public void periodic() {
        innerElevatorPosition = elevatorEncoder.getPosition() + 0.01;
        motorSpeed = elevatorPID.calculate(innerElevatorPosition) + 0.05;
        //motorSpeed = elevator
        if(innerElevatorPosition >= InnerElevatorStates.MAX.height || innerElevatorPosition <= InnerElevatorStates.MIN.height) {
            // positive goes up 
            elevatorMotor.set(0.0);
            inBounds = false;
        } else {
            //motorSpeed = elevatorPID.calculate(innerElevatorPosition);
            elevatorMotor.set(0);
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
    }
}