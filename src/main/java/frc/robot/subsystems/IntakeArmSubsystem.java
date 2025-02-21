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
import frc.robot.States.ArmStates;

public class IntakeArmSubsystem extends SubsystemBase {   
    private final TalonFX armMotor;
    private final SparkMax indexingMotor;
    private final DutyCycleEncoder armEncoder; 
    private final ArmFeedforward armFeedforward;
    private final PIDController armPID;
    private ArmStates armState;
    private double armPosition;
    private double motorSpeed; 
    private boolean override;

    public IntakeArmSubsystem() {
        armMotor = new TalonFX(Constants.Arm.armMotorID);
        indexingMotor = new SparkMax(Constants.Arm.indexingMotorID, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(Constants.Arm.encoderID);

        armFeedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);
        armPID = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);

        armState  = ArmStates.STARTING_POSITION;
        override = false;
    }

    @Override
    public void periodic() {
        armPosition = Units.rotationsToDegrees(armEncoder.get() - Units.degreesToRotations(87));

        // position motor speed moves up 
        if(override) {
            armMotor.set(motorSpeed);
        }
        else if(armPosition >= ArmStates.MAX.angle || armPosition <= ArmStates.MIN.angle) {
            armMotor.set(0);
        } else {
            motorSpeed = armPID.calculate(armPosition) + armFeedforward.calculate(Units.degreesToRadians(armPosition), armMotor.getVelocity().getValueAsDouble());
            armMotor.set(motorSpeed);
        }

        setSmartdashboard();
    }

    public void setArmSpeed(double speed, boolean activate) {
        this.override = activate;
        this.motorSpeed = speed;
    }

    public void setArmAngle(double angle) {
        armPID.setSetpoint(angle);
    }

    public void setIntakeSpeed(double speed) {
        indexingMotor.set(speed);
    }
    
    private void setSmartdashboard() {
        SmartDashboard.putString("Arm Subsystem arm state ", armState.toString());
        SmartDashboard.putNumber("Arm Subsystem position", armPosition);
        SmartDashboard.putNumber("Arm Subsystem motor speed", motorSpeed);
        SmartDashboard.putNumber("Arm Subsystem arm position goal", armState.angle);
        SmartDashboard.putNumber("arm Subsytem velocity", indexingMotor.getEncoder().getVelocity());
    }
}