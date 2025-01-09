package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopSwerve extends Command {
    private CommandSwerveDrivetrain m_swerveDrivetrain;
    private Supplier<Double> translationSupplier;
    private Supplier<Double> strafeSupplier;
    private Supplier<Double> rotationSupplier;
    private SwerveRequest.FieldCentric driveController;

    private double translationVal;
    private double strafeVal;
    private double rotationVal;


    public TeleopSwerve(CommandSwerveDrivetrain swerveDrivetrain, Supplier<Double> forward, Supplier<Double> strafe, Supplier<Double> rotation, SwerveRequest.FieldCentric driveController) {
        // Configure additional PID options by calling getController() and setting the desired options
        this.m_swerveDrivetrain = swerveDrivetrain;
        this.translationSupplier = forward;
        this.strafeSupplier = strafe;
        this.rotationSupplier = rotation;
        this.driveController = driveController;
        addRequirements(m_swerveDrivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        translationVal = translationSupplier.get();
        strafeVal = strafeSupplier.get();
        rotationVal = rotationSupplier.get();

        translationVal = f(translationVal) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        strafeVal = f(strafeVal) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        rotationVal = f(rotationVal) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        m_swerveDrivetrain.applyRequest(() -> driveController.withVelocityX(translationVal).withVelocityY(strafeVal).withRotationalRate(rotationVal));
    }
    
    public double f(double x) {
        return Math.pow(x, 3) * 0.795903 + 0.203938 * x;
    }
}
