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

public class InnerElevatorSubsystem extends SubsystemBase{
    private final SparkFlex elevatorMotor;
    private final PIDController elevatorPID;
    private final ElevatorFeedforward elevatorFeedforward;
    private final RelativeEncoder elevatorEncoder;

    private double motorOutput;
    private boolean inBounds;

    public InnerElevatorSubsystem() {
        elevatorMotor = new SparkFlex(InnerElevatorConsts.elevatorMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorFeedforward = new ElevatorFeedforward(InnerElevatorConsts.kS, InnerElevatorConsts.kG, InnerElevatorConsts.kV);
        elevatorPID = new PIDController(InnerElevatorConsts.kP, InnerElevatorConsts.kI, InnerElevatorConsts.kD);
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
}
