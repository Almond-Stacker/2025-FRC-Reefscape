package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.AlgaeIntakeStates;
import frc.robot.States.ArmStates;
import frc.robot.States.IndexStates;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {
    private final AlgaeIntakeSubsystem AlgaeIntakeSubsystem;
    private final AlgaeIntakeStates indexState;

    public AlgaeIntakeCommand(AlgaeIntakeSubsystem AlgaeIntakeSubsystem, AlgaeIntakeCommandConfiguration config) {
        this.AlgaeIntakeSubsystem = AlgaeIntakeSubsystem;
        this.indexState = config.indexState;
        addRequirements(AlgaeIntakeSubsystem);
    }

    @Override
    public void initialize() {
        if(indexState != null){
            AlgaeIntakeSubsystem.setAlgaeIntakeState(indexState);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    

    public static class AlgaeIntakeCommandConfiguration {
        private AlgaeIntakeStates indexState;

        public AlgaeIntakeCommandConfiguration() {
            this.indexState = null;
        }

        public AlgaeIntakeCommandConfiguration withIndexState(AlgaeIntakeStates indexState) {
            this.indexState = indexState;
            return this;
        }

        public AlgaeIntakeCommandConfiguration build() {
            return this;
        }
    }
}
