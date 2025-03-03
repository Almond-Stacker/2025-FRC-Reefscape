package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConsts;
import frc.robot.commands.ClimbCommand;

public class ClimbSubsystem extends SubsystemBase{
    private final SparkFlex climbMotor;
    private final ClimbCommand commands;

    public ClimbSubsystem() {
        climbMotor = new SparkFlex(27, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(climbMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);

        commands = new ClimbCommand(this);
    }
    
    public void setClimb(double speed) {
        climbMotor.set(speed);
    }

    public ClimbCommand getCommand() {
        return commands;
    }

    public void disableSubsystem() {
        climbMotor.disable();
    }
}
