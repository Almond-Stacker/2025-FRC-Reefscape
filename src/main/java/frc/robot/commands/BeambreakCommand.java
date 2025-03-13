package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BeambreakSubsystem;

public class BeambreakCommand extends Command {
    private final BeambreakSubsystem beambreak;
    private final SequentialCommandGroup commandToRun;

    public BeambreakCommand(BeambreakSubsystem beambreakSubsystem, SequentialCommandGroup commandToRun) {
        this.beambreak = beambreakSubsystem;
        this.commandToRun = commandToRun;
    }

    
}
