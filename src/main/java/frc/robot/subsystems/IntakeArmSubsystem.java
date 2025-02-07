package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
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
    private final ProfiledPIDController armPID;

    //solutions or positions of arm is given by (angle, height)
    private double armAngle;
    private double armGoalAngle;
    private boolean inBounds;
    private double motorSpeed;
    private double lastSpeed = 0;//idk where to initialize
    private double timeStamp = Timer.getFPGATimestamp();//last time stamp

    private ArmStates armState;
    private IndexStates indexState;

    public IntakeArmSubsystem() {
        ArmMotor = new TalonFX(Constants.Arm.armMotorID);
        indexingMotor = new SparkMax(Constants.Arm.indexingMotorID, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(Constants.Arm.encoderID);

        armFeedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);
        armPID = new ProfiledPIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD, Constants.Arm.ANGLE_CONSTRAINTS); 
        armState = ArmStates.STARTING_POSITION;
        indexState = IndexStates.STOP;
        setArmState(armState);
        setIndexState(indexState);
    }

    @Override
    public void periodic() {
        goToGoalPoint(armGoalAngle);

        setSmartdashboard();
    }
    //just move to goal point duh
    public void goToGoalPoint(double goalAngle) {
        armAngle = Units.rotationsToDegrees(armEncoder.get() - Units.degreesToRotations(87));
        //instantiate here if don't need to print
        motorSpeed = armPID.calculate(armAngle, goalAngle);
        inBounds = false;

        if(armAngle >= ArmStates.MAX.angle || armAngle <= ArmStates.MIN.angle) {
            //ArmMotor.set(0);
        } else {
            //calculate velocity using position and time stamp because it could be throwin it off
            //ArmMotor.set(motorSpeed + armFeedforward.calculate(armPID.getSetpoint().velocity, (armPID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - timeStamp)));
            inBounds = true;
        }

        timeStamp = Timer.getFPGATimestamp();
    }

    public void setArmState(ArmStates state) {
        this.armState = state;
        //goal and setpoint differs in ProfiledPID and PID
        armGoalAngle = state.angle;
    }

    public void setIndexState(IndexStates state) {
        indexingMotor.set(state.speed);
        this.indexState = state;
    }

    public void setSmartdashboard() {
        SmartDashboard.putString("Arm Subsytem index state", indexState.toString());
        SmartDashboard.putString("Arm Subsystem arm state ", armState.toString());
        SmartDashboard.putNumber("Arm Subsystem position", armAngle);
        SmartDashboard.putNumber("Arm Subsystem motor speed", motorSpeed);
        SmartDashboard.putBoolean("Arm Subsystem inbounds", inBounds);
        SmartDashboard.putNumber("Arm Subsystem arm position goal", armState.angle);
    }
    /*
    private IndexStates indexState;
    private ArmStates armState;
    private double armPosition;
    private double motorSpeed; 
    private boolean inBounds; 

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
        inBounds = false;
        if(armPosition >= ArmStates.MAX.angle || armPosition <= ArmStates.MIN.angle) {
            // posotive is up
            ArmMotor.set(motorSpeed);
        } else {
            ArmMotor.set(motorSpeed);
            inBounds = true;
        }
        setSmartdashboard();
    }

    //calculate total desired height by 
    //adding the 2 heights of inner and primary heights


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
        SmartDashboard.putBoolean("Arm Subsystem inbounds", inBounds);
        SmartDashboard.putNumber("Arm Subsystem arm position goal", armState.angle);
    }
        */
}