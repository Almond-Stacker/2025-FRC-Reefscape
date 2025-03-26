package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
    private final DigitalInput beamBreak;

    private boolean beamBroken;
    private Timer brokenTimer;

    public BeamBreakSubsystem() {
        beamBreak = new DigitalInput(2);
        beamBroken = false;

        this.brokenTimer = new Timer();
        this.brokenTimer.start();
    }

    @Override
    public void periodic() {
        if (beamBreak.get()) {
            beamBroken = false;
            brokenTimer.reset();
        } else {
            beamBroken = true;
        }

        SmartDashboard.putBoolean("Beam Broken", beamBroken);
    }

    public double getBrokenTime() {
        return brokenTimer.get();
    }

    public boolean getBeamBroken() {
        return beamBroken;
    }
}
