package frc.robot;

import static edu.wpi.first.units.Units.Horsepower;

import frc.robot.States.AlgaeIntakeStates;
import frc.robot.States.ArmStates;
import frc.robot.States.InnerElevatorStates;
import frc.robot.States.PrimaryElevatorStates;
import frc.robot.States.IndexStates;
import frc.robot.States.AlgaeIntakeStates;

import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.InnerElevatorCommand;
import frc.robot.commands.PrimaryElevatorCommand;
import frc.robot.commands.IntakeArmCommand;

import frc.robot.subsystems.InnerElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PrimaryElevatorSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class CommandFactory {
    public static class InnerElevatorCommandFactory {
        private final InnerElevatorSubsystem elevator;

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

        public InnerElevatorCommand createL4Command() {
            return new InnerElevatorCommand(elevator, new InnerElevatorCommand.InnerElevatorCommandConfiguration()
                .withInnerElevator(InnerElevatorStates.L4).build());
        } 
    }

    public static class PrimaryElevatorCommandFactory {
        private final PrimaryElevatorSubsystem elevator;

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

        public PrimaryElevatorCommand createL4Command() {
            return new PrimaryElevatorCommand(elevator, new PrimaryElevatorCommand.PrimaryElevatorCommandConfiguration()
                .withPrimaryElevator(PrimaryElevatorStates.L4).build());
        }
    }

    public static class AlgaeIntakeCommandFactory {
        private final AlgaeIntakeSubsystem intake;

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
        private final IntakeArmSubsystem intake;

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
    }
}
