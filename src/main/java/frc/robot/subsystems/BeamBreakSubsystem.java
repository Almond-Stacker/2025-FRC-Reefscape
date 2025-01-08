package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class BeamBreakSubsystem extends SubsystemBase{
    private DigitalInput BeamBreak;
    private Timer timer;
    private double lastBreak; 
    private double timeSinceLastBreak;
    private boolean isBroken;

    public BeamBreakSubsystem() {
        BeamBreak = new DigitalInput(Constants.BeamBreak.beamBreakID);
        timer = new Timer();
        isBroken = false;
    }
    

    // change to whether break is on 
    @Override
    public void periodic() {
        if(BeamBreak.get()) {
            isBroken = true;
            lastBreak = timer.get();
        } else {
            isBroken = false;
        }
        timeSinceLastBreak = timer.get() - lastBreak;
    }

    public boolean isBroken() {
        return isBroken;
    }
    
}
