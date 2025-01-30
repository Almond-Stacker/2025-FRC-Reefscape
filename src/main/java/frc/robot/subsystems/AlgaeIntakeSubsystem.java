package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants;
import frc.robot.States.AlgaeIntakeStates;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkFlex intakeMotor;
    private AlgaeIntakeStates state;

    public AlgaeIntakeSubsystem() {
        this.state = AlgaeIntakeStates.STOP;
        intakeMotor = new SparkFlex(Constants.AlgaeIntake.intakeMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(intakeMotor, SparkFlexUtil.Usage.kVelocityOnly, IdleMode.kBrake, false, true);
        setAlgaeIntakeState(state);
    }

    public void setAlgaeIntakeState(AlgaeIntakeStates state) {
        this.state = state;
        intakeMotor.set(0);
    }

    private void setSmartdashboard() {

    }
}