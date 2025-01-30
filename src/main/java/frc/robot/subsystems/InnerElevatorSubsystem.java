package frc.robot.subsystems;

import javax.print.DocFlavor.INPUT_STREAM;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants;
import frc.robot.States.InnerElevatorStates;

public class InnerElevatorSubsystem extends SubsystemBase {
    private final SparkFlex elevatorMotor;
    private final PIDController elevatorPID;
    private final AbsoluteEncoder elevatorEncoder;

    private InnerElevatorStates state;
    private double innerElevatorPosition; 
    private double motorSpeed;
    private boolean inBounds;

    public InnerElevatorSubsystem() {
        this.state = InnerElevatorStates.HOME;
        elevatorMotor = new SparkFlex(Constants.InnerElevator.ElevatorMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(elevatorMotor, null, IdleMode.kBrake, false, false);
        elevatorEncoder = elevatorMotor.getAbsoluteEncoder();

        elevatorPID = new PIDController(Constants.InnerElevator.kP, Constants.InnerElevator.kI, Constants.InnerElevator.kD);
    }

    @Override
    public void periodic() {
        innerElevatorPosition = Units.degreesToRadians(elevatorEncoder.getPosition());
        if(innerElevatorPosition >= InnerElevatorStates.MAX.height || innerElevatorPosition <= InnerElevatorStates.MIN.height) {
            elevatorMotor.set(0);
            inBounds = false;
        } else {
            motorSpeed = elevatorPID.calculate(elevatorEncoder.getPosition());
            elevatorMotor.set(motorSpeed);
            inBounds = true; 
        }
    }

    public void setInnerElevatorState(InnerElevatorStates state) {
        this.elevatorPID.setSetpoint(state.height);
        this.state = state;
    }

    private void setSmartdashboard() {
    }
}