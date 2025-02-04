package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.States.ArmStates;
import frc.robot.States.IndexStates;

public class IntakeArmSubsystem extends SubsystemBase {   
    private final TalonFX ArmMotor;
    private final SparkMax indexingMotor;
    private final DutyCycleEncoder armEncoder; 
    private final ArmFeedforward armFeedforward;
    private final PIDController armPID;

    private IndexStates indexState;
    private ArmStates armState;
    private double armPosition;
    private double motorSpeed; 

    public IntakeArmSubsystem() {
        ArmMotor = new TalonFX(Constants.Arm.armMotorID);
        indexingMotor = new SparkMax(Constants.Arm.indexingMotorID, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(Constants.Arm.encoderID);

        armFeedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);
        armPID = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);

        armState  = ArmStates.STARTING_POSITION;
        indexState = IndexStates.STOP;
        setIndexState(indexState);
        setArmState(armState);
    }

    @Override
    public void periodic() {
        armPosition = Units.rotationsToDegrees(armEncoder.get() - Units.degreesToRotations(87));
        motorSpeed = armPID.calculate(armPosition) + armFeedforward.calculate(Units.degreesToRadians(armPosition), ArmMotor.getVelocity().getValueAsDouble());

        if(armPosition >= ArmStates.MAX.angle || armPosition <= ArmStates.MIN.angle) {
            // posotive is up
            ArmMotor.set(0);
        } else {
            ArmMotor.set(motorSpeed);
        }
        setSmartdashboard();
    }

    public void setArmState(ArmStates state) {
        this.armState = state;
        armPID.setSetpoint(armState.angle);
    }

    public void setIndexState(IndexStates state) {
        indexingMotor.set(state.speed);
        this.indexState = state;
    }
    
    private void setSmartdashboard() {
        SmartDashboard.putString("Arm Subsytem index state", indexState.toString());
        SmartDashboard.putString("Arm Subsystem arm state ", armState.toString());
        SmartDashboard.putNumber("Arm Subsystem position", armPosition);
        SmartDashboard.putNumber("Arm Subsystem motor speed", motorSpeed);
        SmartDashboard.putNumber("Arm Subsystem arm position goal", armState.angle);
    }
}