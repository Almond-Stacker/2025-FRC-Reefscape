package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States.AlgaeIntakeStates;
import frc.robot.States.ArmStates;
import frc.robot.States.InnerElevatorStates;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.States.IndexStates;

import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.InnerElevatorCommand;
import frc.robot.commands.PrimaryElevatorCommand;
import frc.robot.commands.sigma;
import frc.robot.commands.IntakeArmCommand;

import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class CommandFactory {
    public static class InnerElevatorCommandFactory {
        public final InnerElevatorSubsystem elevator;

        public InnerElevatorCommandFactory(InnerElevatorSubsystem elevator) {
            this.elevator = elevator;
        }

        public InnerElevatorCommand createHomeCommand() {
            return new InnerElevatorCommand(elevator, new InnerElevatorCommand.InnerElevatorCommandConfiguration()
                .withInnerElevator(InnerElevatorStates.HOME).build());
        } 

        public InnerElevatorCommand createL1Command() {
            return new InnerElevatorCommand(elevator, new InnerElevatorCommand.InnerElevatorCommandConfiguration()
                .withInnerElevator(InnerElevatorStates.L1).build());
        } 

        public InnerElevatorCommand createL2Command() {
            return new InnerElevatorCommand(elevator, new InnerElevatorCommand.InnerElevatorCommandConfiguration()
                .withInnerElevator(InnerElevatorStates.L2).build());
        } 

        public InnerElevatorCommand createL3Command() {
            return new InnerElevatorCommand(elevator, new InnerElevatorCommand.InnerElevatorCommandConfiguration()
                .withInnerElevator(InnerElevatorStates.L3).build());
        } 




































    }

    public static class PrimaryElevatorCommandFactory {
        public final PrimaryElevatorSubsystem elevator;

        public PrimaryElevatorCommandFactory(PrimaryElevatorSubsystem elevator) {
            this.elevator = elevator;
        }

        public PrimaryElevatorCommand createHomeCommand() {
            return new PrimaryElevatorCommand(elevator, new PrimaryElevatorCommand.PrimaryElevatorCommandConfiguration()
                .withPrimaryElevator(PrimaryElevatorStates.HOME).build());
        }

        public PrimaryElevatorCommand createL1Command() {
            return new PrimaryElevatorCommand(elevator, new PrimaryElevatorCommand.PrimaryElevatorCommandConfiguration()
                .withPrimaryElevator(PrimaryElevatorStates.L1).build());
        }

        public PrimaryElevatorCommand createL2Command() {
            return new PrimaryElevatorCommand(elevator, new PrimaryElevatorCommand.PrimaryElevatorCommandConfiguration()
                .withPrimaryElevator(PrimaryElevatorStates.L2).build());
        }

        public PrimaryElevatorCommand createL3Command() {
            return new PrimaryElevatorCommand(elevator, new PrimaryElevatorCommand.PrimaryElevatorCommandConfiguration()
                .withPrimaryElevator(PrimaryElevatorStates.L3).build());
        }

        public PrimaryElevatorCommand createPreIntakeCommand() {
            return new PrimaryElevatorCommand(elevator, new PrimaryElevatorCommand.PrimaryElevatorCommandConfiguration()
               .withPrimaryElevator(PrimaryElevatorStates.PRE_INTAKE).build());
        }

        public PrimaryElevatorCommand createIntakeCommand() {
            return new PrimaryElevatorCommand(elevator, new PrimaryElevatorCommand.PrimaryElevatorCommandConfiguration()
                .withPrimaryElevator(PrimaryElevatorStates.INTAKE).build());
        }
    }

    public static class AlgaeIntakeCommandFactory {
        public final AlgaeIntakeSubsystem intake;

        public AlgaeIntakeCommandFactory(AlgaeIntakeSubsystem intake) {
            this.intake = intake;
        }

        public AlgaeIntakeCommand createStopCommand() {
            return new AlgaeIntakeCommand(intake, new AlgaeIntakeCommand.AlgaeIntakeCommandConfiguration()
                .withIndexState(AlgaeIntakeStates.STOP).build());
        }

        public AlgaeIntakeCommand createIntakeCommand() {
            return new AlgaeIntakeCommand(intake, new AlgaeIntakeCommand.AlgaeIntakeCommandConfiguration()
                .withIndexState(AlgaeIntakeStates.INTAKE).build());
        }

        public AlgaeIntakeCommand createFeedOutCommand() {
            return new AlgaeIntakeCommand(intake, new AlgaeIntakeCommand.AlgaeIntakeCommandConfiguration()
                .withIndexState(AlgaeIntakeStates.FEED_OUT).build());
        }
    }

    public static class IntakeArmCommandFactory {
        public final IntakeArmSubsystem intake;

        public IntakeArmCommandFactory(IntakeArmSubsystem intake) {
            this.intake = intake;
        }

        public IntakeArmCommand createHomeCommand() {
            return new IntakeArmCommand(intake, new IntakeArmCommand.IntakeArmCommandConfiguration()
                .withArmState(ArmStates.HOME).build());
        }

        public IntakeArmCommand createIntakeCommand() {
            return new IntakeArmCommand(intake, new IntakeArmCommand.IntakeArmCommandConfiguration()
                .withIndexState(IndexStates.INTAKE).build());
        }

        public IntakeArmCommand createStopCommand() {
            return new IntakeArmCommand(intake, new IntakeArmCommand.IntakeArmCommandConfiguration()
                .withIndexState(IndexStates.STOP).build());
        }

        public IntakeArmCommand createFeedOutCommand() {
            return new IntakeArmCommand(intake, new IntakeArmCommand.IntakeArmCommandConfiguration()
                .withIndexState(IndexStates.FEED_OUT).build());
        }

        public IntakeArmCommand createIntakePositionCommand() {
            return new IntakeArmCommand(intake, new IntakeArmCommand.IntakeArmCommandConfiguration()
                .withArmState(ArmStates.INTAKE));
        }

        public IntakeArmCommand createL1Command() {
            return new IntakeArmCommand(intake, new IntakeArmCommand.IntakeArmCommandConfiguration()
                .withArmState(ArmStates.L1));
        }

        public IntakeArmCommand createL2Command() {
            return new IntakeArmCommand(intake, new IntakeArmCommand.IntakeArmCommandConfiguration()
                .withArmState(ArmStates.L2));
        }

        public IntakeArmCommand createL3Command() {
            return new IntakeArmCommand(intake, new IntakeArmCommand.IntakeArmCommandConfiguration()
                .withArmState(ArmStates.L3));
        }

        public IntakeArmCommand createStartingPositionCommand() {
            return new IntakeArmCommand(intake, new IntakeArmCommand.IntakeArmCommandConfiguration()
                .withArmState(ArmStates.STARTING_POSITION));       
        }
    }

    public static class CombinationCommandFactory {
        private final IntakeArmCommandFactory intakeFactory;
        private final AlgaeIntakeCommandFactory algaeFactory;
        private final PrimaryElevatorCommandFactory primaryElevatorFactory;
        private final InnerElevatorCommandFactory innerElevatorFactory; 

        public CombinationCommandFactory(IntakeArmCommandFactory intake, AlgaeIntakeCommandFactory algae, PrimaryElevatorCommandFactory elevator, InnerElevatorCommandFactory innerElevator) {
            intakeFactory = intake;
            algaeFactory = algae;
            primaryElevatorFactory = elevator;
            innerElevatorFactory = innerElevator;
        }

        public SequentialCommandGroup createScoreL1Command() {
            return new SequentialCommandGroup(intakeFactory.createL1Command(),
                                               new WaitCommand(0.3),
                                                primaryElevatorFactory.createL1Command(),
                                                innerElevatorFactory.createL1Command());
        }

        public SequentialCommandGroup createScoreL2Command() {
            return new SequentialCommandGroup(intakeFactory.createL2Command(),
                                                new WaitCommand(0.3),
                                                primaryElevatorFactory.createL2Command(),
                                                innerElevatorFactory.createL2Command());
        }

        
        public SequentialCommandGroup createScoreL3Command() {
            return new SequentialCommandGroup(intakeFactory.createL3Command(),
                                               new WaitCommand(0.3),
                                                primaryElevatorFactory.createL3Command(),
                                                innerElevatorFactory.createL3Command());
        }

        public SequentialCommandGroup createPreIntakeCommand() {
            return new SequentialCommandGroup(primaryElevatorFactory.createPreIntakeCommand(),
                                                intakeFactory.createIntakePositionCommand());
        }

        public SequentialCommandGroup createHomeCommand() {
            return new SequentialCommandGroup(primaryElevatorFactory.createHomeCommand(),
                                                innerElevatorFactory.createHomeCommand(),
                                                intakeFactory.createStartingPositionCommand());
        }

        public SequentialCommandGroup createFeedOutScoreCommand(sigma test) {
            return new SequentialCommandGroup(new InstantCommand( () -> intakeFactory.intake.setArmSpeed(-0.1, true)),
                                                //innerElevatorFactory.createL3Score(),
                                                new WaitCommand(0.5),
                                                intakeFactory.createFeedOutCommand(), 
                                                new WaitCommand(0.3),
                                                intakeFactory.createStopCommand(),
                                                new WaitCommand(0.1),
                                                test,
                                                new WaitCommand(0.7),
                                                new InstantCommand(() -> intakeFactory.intake.setArmSpeed(0, false)));

        }

        public SequentialCommandGroup createIntakeCommand() {
            return new SequentialCommandGroup(primaryElevatorFactory.createIntakeCommand(),
                                                //innerElevatorFactory.createIntakeCommand(),
                                                intakeFactory.createIntakeCommand(),
                                                new WaitCommand(0.6),
                                                intakeFactory.createStopCommand());
        }

    }
}
