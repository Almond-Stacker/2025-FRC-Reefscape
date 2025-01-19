package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.States.ElevatorStates;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private ElevatorStates state;
    private PIDController elevatorPID;
    private ElevatorFeedforward elevatorFeedforward; 
    private TalonFX motor; 
    private Encoder encoder;

    private double pidOutput;
    private double feedForwardOutput;

    public Elevator(ElevatorStates state) {
        this.state = state;
        elevatorPID = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
        elevatorFeedforward = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV);

        initalizeMotors();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        motor.set(elevatorPID.calculate(encoder.getDistance()) + elevatorFeedforward.calculate(Constants.Elevator.velocitySetPoint));
    }

    public void setElevatorState(ElevatorStates state) {
        this.state = state;
        elevatorPID.setSetpoint(state.height);
    }

    private void initalizeMotors() {
        motor = new TalonFX(Constants.Elevator.elevatorMotorID);
        encoder = new Encoder(0, 0);
    }
    
    private void setSmartdashboardData() {
        SmartDashboard.putString("Elevator State", state.toString());
        SmartDashboard.putNumber("Elevator PID output", pidOutput);
        SmartDashboard.putNumber("Elevator Feedforward output", feedForwardOutput);
    }
}
