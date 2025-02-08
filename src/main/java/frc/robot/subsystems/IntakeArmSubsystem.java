package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConsts;
import frc.robot.States.IntakeArmStates;
import frc.robot.commands.IntakeArmCommand;

public class IntakeArmSubsystem extends SubsystemBase{
    private final TalonFX armMotor;
    private final SparkMax suckMotor;
    private final DutyCycleEncoder armEncoder;
    private final ArmFeedforward armFeedforward;
    private final ProfiledPIDController armPID;

    //solution pairs (height, theta) calculated in states
    private boolean inBounds;
    private double motorOutput;
    
    private double lastSpeed = 0;
    private double timeStamp = Timer.getFPGATimestamp();

    private IntakeArmCommand commands;

    public IntakeArmSubsystem() {
        armMotor = new TalonFX(IntakeArmConsts.armMotorID);
        suckMotor = new SparkMax(IntakeArmConsts.suckMotorID, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(IntakeArmConsts.encoderID);

        armFeedforward = new ArmFeedforward(IntakeArmConsts.kS, IntakeArmConsts.kG, IntakeArmConsts.kV);
        armPID = new ProfiledPIDController(IntakeArmConsts.kP, IntakeArmConsts.kI, IntakeArmConsts.kD, IntakeArmConsts.ANGLE_CONSTRAINTS);
    }

    @Override
    public void periodic() {
        double armAngle = Units.rotationsToDegrees(armEncoder.get() - Units.degreesToRotations(87));

        if(armAngle >= IntakeArmStates.MAX.angle
                || armAngle <= IntakeArmStates.MIN.angle) {
                    armMotor.set(0);
                    inBounds = false;
        } else {
            motorOutput = armPID.calculate(getAngle()) + armFeedforward.calculate(armPID.getSetpoint().velocity, (armPID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - timeStamp));
        }
        
        lastSpeed = armPID.getSetpoint().velocity;
        timeStamp = Timer.getFPGATimestamp();
    }

    public void setAngle(double angle) {

    }

    public void reset() {

    }

    public boolean atAngle() {

    }

    public double getAngle() {

    }

    private void setSmartdashboard() {

    }

    public IntakeArmCommand getCommands() {
        return commands;
    }
}
