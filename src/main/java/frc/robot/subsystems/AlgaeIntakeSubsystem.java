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
import frc.robot.States.AlgaeIntake;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkFlex intakeMotor;
    private AlgaeIntake state;

    public AlgaeIntakeSubsystem() {
        this.state = AlgaeIntake.HOME;
        intakeMotor = new SparkFlex(Constants.AlgaeIntake.intakeMotorID, MotorType.kBrushless);
        SparkFlexUtil.setSparkFlexBusUsage(intakeMotor, null, IdleMode.kBrake, false, false);
    }

    public void setAlgaeIntakeState(AlgaeIntake state) {
        this.state = state;
        intakeMotor.set(state.angle);
    }

    private void setSmartdashboard() {

    }
}