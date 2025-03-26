package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
    private final DigitalInput beamBreak;

    private boolean beamBroken;
    private double brokenTimer;


    public BeamBreakSubsystem() {
        beamBreak = new DigitalInput(2);
        beamBroken = false;


        this.brokenTimer = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        if (beamBreak.get()) {
            beamBroken = false;
            brokenTimer = Timer.getFPGATimestamp();
        } else {
            beamBroken = true;
        }

        SmartDashboard.putBoolean("Beam Broken", beamBroken);
    }

    public double getBrokenTime() {
        return Timer.getFPGATimestamp() - brokenTimer;
    }

    public boolean getBeamBroken() {
        return beamBroken;
    }
}
