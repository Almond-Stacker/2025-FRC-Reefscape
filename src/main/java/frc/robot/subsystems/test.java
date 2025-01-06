// package frc.robot.subsystems;
// import static edu.wpi.first.units.Units.*;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.SignalLogger;
// import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.swerve.SwerveDrivetrain;
// import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import choreo.Choreo.TrajectoryLogger;
// import choreo.auto.AutoFactory;
// import choreo.auto.AutoFactory.AutoBindings;
// import com.choreo.trajectory.SwerveSample;
// import com.choreo.trajectory.SwerveSample;



// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// /**
//  * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
//  * Subsystem so it can easily be used in command-based projects.
//  */
// public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
//     private static final double kSimLoopPeriod = 0.005; // 5 ms
//     private Notifier m_simNotifier = null;
//     private double m_lastSimTime;

//     /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
//     private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
//     /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
//     private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
//     /* Keep track if we've ever applied the operator perspective before or not */
//     private boolean m_hasAppliedOperatorPerspective = false;

//     /** Swerve request to apply during field-centric path following */
//     private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
//     private final PIDController m_pathXController = new PIDController(10, 0, 0);
//     private final PIDController m_pathYController = new PIDController(10, 0, 0);
//     private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

//     /* Swerve requests to apply during SysId characterization */
//     private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
//     private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
//     private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

//     public CommandSwerveDrivetrain(
//     /**
//      * Creates a new auto factory for this drivetrain.
//      *
//      * @return AutoFactory for this drivetrain
//      */
//     public AutoFactory createAutoFactory() {
//         return createAutoFactory(new AutoBindings());
//     }

//     /**
//      * Creates a new auto factory for this drivetrain with the given
//      * global auto bindings.
//      *
//      * @param autoBindings Global bindings to apply to the factory
//      * @return AutoFactory for this drivetrain
//      */
//     public AutoFactory createAutoFactory(AutoBindings autoBindings) {
//         return createAutoFactory(autoBindings, (sample, isStart) -> {});
//     }

//     /**
//      * Creates a new auto factory for this drivetrain with the given
//      * global auto bindings and trajectory logger.
//      *
//      * @param autoBindings Global bindings to apply to the factory
//      * @param trajLogger Logger for the trajectory
//      * @return AutoFactory for this drivetrain
//      */
//     public AutoFactory createAutoFactory(AutoBindings autoBindings, TrajectoryLogger<SwerveSample> trajLogger) {
//         return new AutoFactory(
//             () -> getState().Pose,
//             this::resetPose,
//             this::followPath,
//             true,
//             this,
//             autoBindings,
//             trajLogger
//         );
//     }

//     /**
//      * Returns a command that applies the specified control request to this swerve drivetrain.
//      *
//      * @param request Function returning the request to apply
//      * @return Command to run
//      */
//     public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
//         return run(() -> this.setControl(requestSupplier.get()));
//     }

//     /**
//      * Follows the given field-centric path sample with PID.
//      *
//      * @param sample Sample along the path to follow
//      */
//     public void followPath(SwerveSample sample) {
//         m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

//         var pose = getState().Pose;

//         var targetSpeeds = sample.getChassisSpeeds();
//         targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
//             pose.getX(), sample.x
//         );
//         targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
//             pose.getY(), sample.y
//         );
//         targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
//             pose.getRotation().getRadians(), sample.heading
                        //         );

//         setControl(
//             m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
//                 .withWheelForceFeedforwardsX(sample.moduleForcesX())
//                 .withWheelForceFeedforwardsY(sample.moduleForcesY())
//         );
//     }
// }