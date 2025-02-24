package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.States.IntakeArmStates;
import frc.robot.commands.IntakeArmCommand;

public class IntakeArmSubsystem extends SubsystemBase {   
    private final TalonFX armMotor;
    private final SparkMax indexingMotor;
    private final DutyCycleEncoder armEncoder; 
    private final ArmFeedforward armFeedforward;
    private final PIDController armPID;
    private final IntakeArmCommand command;

    private double armPosition;
    private double motorSpeed; 

    public IntakeArmSubsystem() {
        armMotor = new TalonFX(Constants.Arm.armMotorID);
        indexingMotor = new SparkMax(Constants.Arm.indexingMotorID, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(Constants.Arm.encoderID);

        armFeedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);
        armPID = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);
        command = new IntakeArmCommand(this);
    }

    @Override
    public void periodic() {
        armPosition = Units.rotationsToDegrees(armEncoder.get() - Units.degreesToRotations(87));

        if(armPosition >= IntakeArmStates.MAX.angle || armPosition <= IntakeArmStates.MIN.angle) {
            armMotor.set(0);
            return;
        } 
        
        motorSpeed = armPID.calculate(armPosition) + armFeedforward.calculate(Units.degreesToRadians(armPosition), armMotor.getVelocity().getValueAsDouble());
        armMotor.set(motorSpeed);
        setSmartdashboard();
    }

    public void setArmAngle(double angle) {
        armPID.setSetpoint(angle);
    }

    public void setIntakeSpeed(double speed) {
        indexingMotor.set(speed);
    }
    
    private void setSmartdashboard() {
        SmartDashboard.putNumber("Arm subsystem current angle;", armPosition);
        SmartDashboard.putNumber("Arm subsystem motor speed", motorSpeed);    
        SmartDashboard.putNumber("Arm subsystem goal angle", command.getIntakeArmState().angle);
        SmartDashboard.putNumber("Arm subsystem intake speed", command.getIntakeState().speed);

        SmartDashboard.putString("Arm subsystem arm state", command.getIntakeArmState().toString());
        SmartDashboard.putString("Arm subsystem intake state", command.getIntakeState().toString());
    }
}