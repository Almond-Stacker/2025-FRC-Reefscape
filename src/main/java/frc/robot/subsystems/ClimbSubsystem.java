package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConsts;
import frc.robot.States.ClimbStates;


public class ClimbSubsystem extends SubsystemBase{
    private final SparkFlex climbMotor;
    private final DutyCycleEncoder climbEncoder;
    private double climbSpeed;

    public ClimbSubsystem() {
        climbMotor = new SparkFlex(ClimbConsts.climbMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(climbMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, true);
        climbEncoder = new DutyCycleEncoder(ClimbConsts.encoderID);
        climbSpeed = 0; 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        climbMotor.set(climbSpeed);
        if(climbEncoder.get() < Constants.ClimbConsts.maxThreshold){
            climbSpeed = 0.1;
        } else if (climbEncoder.get() > Constants.ClimbConsts.minThreshold){
            climbSpeed = -0.1;
        }
    }

    public void setClimbState(ClimbStates climbState) {
        //climbMotor.set(climbState.speed);
        climbSpeed = climbState.speed;
    }

}
