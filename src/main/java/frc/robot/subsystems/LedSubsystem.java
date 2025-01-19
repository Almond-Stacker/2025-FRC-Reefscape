package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.States.ledStates;

public class LedSubsystem extends SubsystemBase{
    private Spark led;

    public LedSubsystem() {
        intializeLED();
        setLedState(ledStates.BLACK);
    }

    public void setLedState(ledStates state) {
        led.set(state.value);
    }

    private void intializeLED() {
        led = new Spark(Constants.LED.LED_ID);
    }

}
