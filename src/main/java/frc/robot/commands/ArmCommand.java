package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.States.ArmStates;
import frc.robot.States.IndexStates;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final IndexStates indexState;
    private final ArmStates armState;

    public ArmCommand(ArmSubsystem armSubsystem, ArmCommandConfiguration config) {
        this.armSubsystem = armSubsystem;
        this.indexState = config.indexState;
        this.armState = config.armState;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        if(indexState != null){
            armSubsystem.setIndexState(indexState);
        }

        if(armState != null){
            armSubsystem.setArmState(armState);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    

    public static class ArmCommandConfiguration {
        private IndexStates indexState;
        private ArmStates armState;

        public ArmCommandConfiguration() {
            this.indexState = null;
            this.armState = null;
        }

        public ArmCommandConfiguration withIndexState(IndexStates indexState) {
            this.indexState = indexState;
            return this;
        }

        public ArmCommandConfiguration withArmState(ArmStates armState) {
            this.armState = armState;
            return this;
        }

        public ArmCommandConfiguration build() {
            return this;
        }
    }
}
