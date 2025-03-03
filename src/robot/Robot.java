package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
    //private Command m_autonomousCommand;
    //instantiate by getting autonomousCommand in robotContainer

    private final RobotContainer m_robotContainer;

    public Robot() {

        //Will perform all button bindings
        m_robotContainer = new RobotContainer();

        //calculate and load all apriltag map data here
        
        //after put into hashmap held in constants for 
        //more generalized constants before periodic
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        //subsystems, then button polls, then commands, then end commands
    }
    
    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void autonomousInit() {
        
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
