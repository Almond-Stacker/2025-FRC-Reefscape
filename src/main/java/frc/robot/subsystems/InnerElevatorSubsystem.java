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
import frc.robot.Constants.InnerElevator;
import frc.robot.States.InnerElevatorStates;
import frc.robot.commands.InnerElevatorCommand;
import frc.robot.commands.InnerElevatorCommand;

public class InnerElevatorSubsystem extends SubsystemBase {
    private final SparkFlex elevatorMotor;
    private final RelativeEncoder encoder;
    
    private final PIDController elevatorPID;
    private final ElevatorFeedforward elevatorFeedforward;
    private final RelativeEncoder elevatorEncoder;
    private final InnerElevatorCommand elevatorCommand;
    private double innerElevatorPosition; 
    private double motorSpeed;
    private boolean inBounds;


    public InnerElevatorSubsystem() {
        elevatorMotor = new SparkFlex(Constants.InnerElevator.ElevatorMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
        elevatorFeedforward = new ElevatorFeedforward(Constants.InnerElevator.kS, Constants.InnerElevator.kG, Constants.InnerElevator.kV);
        encoder = elevatorMotor.getEncoder();
        elevatorPID = new PIDController(Constants.InnerElevator.kP, Constants.InnerElevator.kI, Constants.InnerElevator.kD);
        elevatorCommand = new InnerElevatorCommand(this);
        override = false;
    }

    @Override
    public void periodic() {
        innerElevatorPosition = elevatorEncoder.getPosition() + 0.1;
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

            //elevatorMotor.set(motorSpeed);
            elevatorMotor.set(0);
            inBounds = true; 
        }
        setSmartdashboard();
    }

    public void setElevatorHeight(double height) {
        this.elevatorPID.setSetpoint(height);
    }

    public void setInnerElevatorSpeed(double speed) {
        elevatorMotor.set(speed);
    }

    public InnerElevatorCommand getCommand() {
        return elevatorCommand;
    }

    private void setSmartdashboard() {
        SmartDashboard.putBoolean("Inner elevator in bounds", inBounds);
        SmartDashboard.putNumber("Inner elevator speed", motorSpeed);
        SmartDashboard.putNumber("Inner elevator posotion ", innerElevatorPosition);
    }
}