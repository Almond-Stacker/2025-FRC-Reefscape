// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.States.IndexStates;
// import frc.robot.subsystems.ArmSubsystem;

// public class IndexCommand {
//     private final ArmSubsystem armSubsystem;
//     private IndexStates state;

//     public IndexCommand(ArmSubsystem armSubsystem) {
//         this.armSubsystem = armSubsystem;
//     }

//     public InstantCommand setIndexState(IndexStates state) {
//         this.state = state;
//         return new InstantCommand(() -> armSubsystem.setIndexState(state), armSubsystem);
//     }

//     public IndexStates getCurrentState() {
//         return state;
//     }
// }
