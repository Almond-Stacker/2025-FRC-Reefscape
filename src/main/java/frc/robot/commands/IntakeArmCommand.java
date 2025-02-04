package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.States.ArmStates;
import frc.robot.States.IndexStates;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmCommand extends Command {
    private final IntakeArmSubsystem IntakeArmSubsystem;
    private final IndexStates indexState;
    private final ArmStates armState;

    public IntakeArmCommand(IntakeArmSubsystem IntakeArmSubsystem, IntakeArmCommandConfiguration config) {
        this.IntakeArmSubsystem = IntakeArmSubsystem;
        this.indexState = config.indexState;
        this.armState = config.armState;
        addRequirements(IntakeArmSubsystem);
    }

    @Override
    public void initialize() {
        if(indexState != null){
           IntakeArmSubsystem.setIndexState(indexState);
        }

        if(armState != null){
            IntakeArmSubsystem.setArmState(armState);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    

    public static class IntakeArmCommandConfiguration {
        private IndexStates indexState;
        private ArmStates armState;

        public IntakeArmCommandConfiguration() {
            this.indexState = null;
            this.armState = null;
        }

        public IntakeArmCommandConfiguration withIndexState(IndexStates indexState) {
            this.indexState = indexState;
            return this;
        }

        public IntakeArmCommandConfiguration withArmState(ArmStates armState) {
            this.armState = armState;
            return this;
        }

        public IntakeArmCommandConfiguration build() {
            return this;
        }
    }
}
