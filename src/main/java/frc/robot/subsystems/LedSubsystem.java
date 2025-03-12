package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.States.LedStates;

public class LedSubsystem extends SubsystemBase{
    private final Spark ledController; 
    private LedStates state; 

    public LedSubsystem() {
        ledController = new Spark(Constants.ledControllerID);
        state = LedStates.OFF;
    }

    @Override
    public void periodic() {
        ledController.set(state.frequency);
    }

    public void setLedState(LedStates state) {
        this.state = state;
    }
    
}
