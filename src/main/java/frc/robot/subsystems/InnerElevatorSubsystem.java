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
import frc.robot.commands.InnerElevatorCommand;

public class InnerElevatorSubsystem extends SubsystemBase {
    private final SparkFlex elevatorMotor;
    private final RelativeEncoder encoder;
    
    private final PIDController elevatorPID;
    private final ElevatorFeedforward elevatorFeedforward;
    
    private final InnerElevatorCommand command;

    private double position;
    private double motorSpeed;
    private boolean inBounds;


    public InnerElevatorSubsystem() {
        elevatorMotor = new SparkFlex(Constants.InnerElevator.ElevatorMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
        elevatorFeedforward = new ElevatorFeedforward(Constants.InnerElevator.kS, Constants.InnerElevator.kG, Constants.InnerElevator.kV);
        encoder = elevatorMotor.getEncoder();
        elevatorPID = new PIDController(Constants.InnerElevator.kP, Constants.InnerElevator.kI, Constants.InnerElevator.kD);

        command = new InnerElevatorCommand(this);
    }

    @Override
    public void periodic() {
        // ensure encoder position dosen't become negative 
        position = encoder.getPosition() + 0.01;
        inBounds = false;

        // disable elevator once going outside bounds
        // need to remove before competition or change only here for testing safety 
        if(position >= InnerElevatorStates.MAX.height || position <= InnerElevatorStates.MIN.height) {
            elevatorMotor.set(0.0);
            return;
        } 
        inBounds = true; 

        motorSpeed = elevatorPID.calculate(position) + elevatorFeedforward.calculate(position);
        // compensate for gravity going down 
        // can change but it works 
        if(motorSpeed < -0.1){
            motorSpeed *= 0.2;
        }
        elevatorMotor.set(motorSpeed);

        setSmartdashboard();
    }

    public void setHeight(double height) {
        this.elevatorPID.setSetpoint(height);
    }

    private void setSmartdashboard() {
        SmartDashboard.putNumber("Inner elevator current height", position);
        SmartDashboard.putNumber("Inner elevator motor speed", motorSpeed);
        SmartDashboard.putNumber("Inner elevator goal height", command.getState().height);

        SmartDashboard.putString("Inner elevator state", command.getState().toString());

        SmartDashboard.putBoolean("Inner elevator in bounds", inBounds);
    }
}