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
import frc.robot.commands.InnerElevatorCommand;

public class InnerElevatorSubsystem extends SubsystemBase{
    private final SparkFlex elevatorMotor;
    private final PIDController elevatorPID;
  //  private final ElevatorFeedforward elevatorFeedFoward;
    private final RelativeEncoder elevatorEncoder; 
    private final InnerElevatorCommand command;

    private double motorOutput;
    private boolean inBounds;
    private double currentHeight;

    public InnerElevatorSubsystem() {   
        elevatorMotor = new SparkFlex(Constants.InnerElevatorConsts.elevatorMotorID, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorPID = new PIDController(InnerElevatorConsts.kP, InnerElevatorConsts.kI, InnerElevatorConsts.kD);
        //elevatorFeedFoward = new ElevatorFeedforward(motorOutput, motorOutput, motorOutput);

        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);

        command = new InnerElevatorCommand(this);
    }

    @Override
    public void periodic() {
        currentHeight = getHeight();
        inBounds = false;

        if(currentHeight > ElevatorStates.MAX_ABS.innerHeight) {
            motorOutput = -0.1;
            return;
        } else if(currentHeight < ElevatorStates.MIN_ABS.innerHeight) {
            motorOutput = 0.1;
            return; 
        } else {
            motorOutput = elevatorPID.calculate(getHeight()); //+ elevatorFeedForward.calculate(elevatorPID.getSetpoint(), elevatorMotor.getEncoder().getVelocity());
            inBounds = true;
        }
        
        //elevatorMotor.set(motorOutput);
        setSmartdashboard();
    }

    public void setHeight(double height) {
        elevatorPID.setSetpoint(height);
    }

    public double getRelativeInnerHeight() {
        return (getHeight() - ElevatorStates.MIN_ABS.innerHeight)/(ElevatorStates.MAX_ABS.innerHeight - ElevatorStates.MIN_ABS.innerHeight);
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

    private void disableSubsystem() {
        elevatorMotor.disable();
    }

    private void setSmartdashboard() {
        SmartDashboard.putBoolean("Inner elevator in bounds", inBounds);
        SmartDashboard.putNumber("Inner elevator speed", motorOutput);
        SmartDashboard.putNumber("Inner elevator posotion ", getHeight());
        SmartDashboard.putNumber("Inner elevator Relative position", getRelativeInnerHeight());
        SmartDashboard.putNumber("Inner elevator goal position", elevatorPID.getSetpoint());
    }

    public InnerElevatorCommand getCommands() {
        return command;
    }
}