package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants.InnerElevatorConsts;
import frc.robot.States.InnerElevatorStates;
import frc.robot.commands.InnerElevatorCommand;

public class InnerElevatorSubsystem extends SubsystemBase{
    private final SparkFlex elevatorMotor;
    private final ProfiledPIDController elevatorPID;
    private final ElevatorFeedforward elevatorFeedforward;
    private final RelativeEncoder elevatorEncoder;

    private double motorOutput;
    private boolean inBounds;

    private double lastSpeed = 0;
    private double timeStamp = Timer.getFPGATimestamp();

    private InnerElevatorCommand commands;

    public InnerElevatorSubsystem() {
        elevatorMotor = new SparkFlex(InnerElevatorConsts.elevatorMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorFeedforward = new ElevatorFeedforward(InnerElevatorConsts.kS, InnerElevatorConsts.kG, InnerElevatorConsts.kV);
        elevatorPID = new ProfiledPIDController(InnerElevatorConsts.kP, InnerElevatorConsts.kI, InnerElevatorConsts.kD, InnerElevatorConsts.PROFILE);

        commands = new InnerElevatorCommand(this);
    }

    @Override
    public void periodic() {
        double relativeElevatorPosition = getHeight();

        if(relativeElevatorPosition >= InnerElevatorStates.MAX.height
                || relativeElevatorPosition <= InnerElevatorStates.MIN.height) {
            elevatorMotor.set(0);
            inBounds = false;
        } else {
            motorOutput = elevatorPID.calculate(getHeight()) + elevatorFeedforward.calculate(elevatorPID.getSetpoint().velocity, (elevatorPID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - timeStamp));
            elevatorMotor.set(motorOutput);
            inBounds = true;
        }

        lastSpeed = elevatorPID.getSetpoint().velocity;
        timeStamp = Timer.getFPGATimestamp();
    }

    public void setHeight(double height) {
        elevatorPID.setGoal(height);
    }

    public void reset() {
        elevatorPID.setGoal(getHeight());
        elevatorPID.reset(getHeight());
    }

    public boolean atHeight() {
        return elevatorPID.atGoal();
    }

    public double getHeight() {
        return elevatorEncoder.getPosition() + 0.01;
    }

    private void setSmartdashboard() {
        SmartDashboard.putBoolean("Inner elevator in bounds", inBounds);
        SmartDashboard.putNumber("Inner elevator speed", motorOutput);
        SmartDashboard.putNumber("Inner elevator posotion ", getHeight());
        SmartDashboard.putNumber("Inner elevator goal position", elevatorPID.getGoal().position);
    }

    public InnerElevatorCommand getCommands() {
        return commands;
    }
}
