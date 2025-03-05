package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConsts;
import frc.robot.States.ElevatorStates;
import frc.robot.States.IntakeStates;

public class IntakeArmSubsystem extends SubsystemBase{
    private final TalonFX armMotor;
    private final SparkMax intakeMotor;
    private final DutyCycleEncoder armEncoder;
    private final ArmFeedforward armFeedforward;
    private final PIDController armPID;

    private boolean inBounds;
    private double motorOutput;
    private double currentAngle;
    private double goalPosition;
    private double addMotor;

    public IntakeArmSubsystem() {
        armMotor = new TalonFX(IntakeArmConsts.armMotorID);
        intakeMotor = new SparkMax(IntakeArmConsts.suckMotorID, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(IntakeArmConsts.encoderID);

        armFeedforward = new ArmFeedforward(IntakeArmConsts.kS, IntakeArmConsts.kG, IntakeArmConsts.kV);
        armPID = new PIDController(IntakeArmConsts.kP, IntakeArmConsts.kI, IntakeArmConsts.kD);

        armMotor.setNeutralMode(NeutralModeValue.Brake);
        disableSubsystem();
    }

    @Override
    public void periodic() {
        currentAngle = getCurrentAngle();
        inBounds = false; 

        if(currentAngle > ElevatorStates.MAX.armAngle) {
            motorOutput = -0.05;
        } else if(currentAngle < ElevatorStates.MIN.armAngle) {
            motorOutput = 0.05;
        } else {
            motorOutput = //armPID.calculate(currentAngle); 
                 armFeedforward.calculate(Units.degreesToRadians(currentAngle - 158), armMotor.getVelocity().getValueAsDouble());// (armPID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - timeStamp));
            inBounds = true;
        }

        //armMotor.set(motorOutput + addMotor);
        setSmartdashboard();
    }

    public void setMotorSpeed(double speed) {
        addMotor = speed;
    }

    public void setArmPosition(double armAngle) {
        goalPosition = armAngle;
        armPID.setSetpoint(goalPosition);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    private double getCurrentAngle() {
        return Units.rotationsToDegrees(armEncoder.get()) - 87;
    }

    private void disableSubsystem() {
        armMotor.disable();
        intakeMotor.disable();
    }

    private void setSmartdashboard() {
        SmartDashboard.putNumber("Intake Arm Angle", currentAngle);
        SmartDashboard.putNumber("Intake Arm Motor Output", motorOutput);
        SmartDashboard.putNumber("Intake Arm Goal Angle", goalPosition);

        SmartDashboard.putBoolean("Intake Arm In Bounds", inBounds);
    }
}