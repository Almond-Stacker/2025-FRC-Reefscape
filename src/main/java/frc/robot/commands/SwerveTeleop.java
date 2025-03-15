// package frc.robot.commands;

// import java.util.ResourceBundle.Control;
// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.States.ElevatorStates;

// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class SwerveTeleop extends Command {
//     private final CommandSwerveDrivetrain drivetrain; 
//     private final CommandXboxController controller; 
//     private final Supplier<ElevatorStates> currentState;
//     private final double lowLevelAccelerationConstant; 

//     private double xInput;
//     private double yInput;
//     private double rInput; 

//     public SwerveTeleop(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, Supplier<ElevatorStates> currentState) {
//         this.drivetrain = drivetrain;
//         this.controller = controller;
//         this.currentState = currentState; 

//         xInput = 0; 
//         yInput = 0;
//         rInput = 0;
//     }

//     @Override 
//     public void execute() {
//         if(currentState.get().equals(ElevatorStates.STARTING_POSITION)) {
//             setInputs(lowLevelAccelerationConstant);
//         }
//     }

//     private void setInputs(double acceleration) {
//         if(isInputIncreasing(controller.getLeftX(), xInput)) {
//             xInput += controller.getLeftX() * acceleration;
//         } else {
//             xInput -= controller.getLeftX() * acceleration;
//             if(controller.getLeftX() == 0) {
//                 xInput = Math.min(0, xInput);
//             }
//         }

//         if(isInputIncreasing(controller.getLeftY(), yInput)) {
//             yInput += controller.getLeftY() * acceleration;
//         } else {
//             yInput -= controller.getLeftY() * acceleration;
//             if(controller.getLeftY() == 0) {
//                 yInput = Math.max(0, yInput);
//             }
//         }

//         rInput = controller.getRightX();

//         xInput = Math.min(1, xInput);
//         xInput = Math.max(-1, xInput);

//         yInput = Math.min(1, yInput);
//         yInput = Math.max(-1, yInput);
//     }
    
//     private boolean isInputIncreasing(double currentInput, double previousInput) {
//         if(Math.abs(currentInput - previousInput) > 0) {
//             return true;
//         }

//         if(Math.abs(previousInput) == 1 && Math.abs(currentInput) == 1) {
//             return true; 
//         }

//         return false;
//     }
// }
