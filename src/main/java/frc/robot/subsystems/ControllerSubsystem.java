package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConsts;
import frc.robot.States.ElevatorStates;

public class ControllerSubsystem extends SubsystemBase{
    CommandXboxController driver0;
 //InnerElevatorSubsystem elevatorSubsystem;

    double accX;
    double posX; // controller value
    double rightPosX;

    double accY;
    double posY;

    double accR;
    double posR;

    double slow;

    boolean elevatorUp;
    boolean strafe;
    boolean strafeLeft;
    boolean preciece;

    public ControllerSubsystem(CommandXboxController driver0) {
        this.driver0 = driver0;

        accX = 0;
        posX = 0;
        accY = 0;
        posY = 0;

        accR = 0;
        posR = 0;

        slow = ControllerConsts.SLOW_RATIO;
        elevatorUp = false;
        strafe = false;
        strafeLeft = true;
        preciece = false;

        rightPosX = 0;
    }

    @Override
    public void periodic() {
        // double deadbandRatio = 1;

        rightPosX = getDriverRightX();
        accX = getDriverLeftX();
        accY = getDriverLeftY();
        accR = getDriverRightX();
        // if(!elevatorUp) {
            // deadbandRatio = 1;

        double diffX = posX - accX;
        double diffY = posY - accY;

        double frictionX = ControllerConsts.FRIC + Math.abs(diffX) * 0.3;
        double frictionY = ControllerConsts.FRIC + Math.abs(diffY) * 0.3;
        
            if(Math.abs(accX) >= Math.abs(posX)) {
                posX = accX;
            }
            else {
                posX += (accX - posX) * frictionX;// * precieceFriction;
            }

            if(Math.abs(accY) >= Math.abs(posY)) {
                posY = accY;
            }
            else {
                posY += (accY - posY) * frictionY;
            }

            if(Math.abs(accR) >= Math.abs(posR)) {
                posR = accR;
            }
            else {
                posR += (accR - posR) * ControllerConsts.ROTATIONAL_FRIC;// * precieceFriction;
            }
        // } else {
        //     System.out.println("Elevator down " + posX + " :: " + posY);
        //     deadbandRatio = ControllerConsts.DEADBAND_RATIO;
        //     posX += (accX - posX) * ControllerConsts.FRIC * slow;// * precieceFriction;
        //     posY += (accY - posY) * ControllerConsts.FRIC * slow;// * precieceFriction;
        // }
        
        
        
            // posX = Math.max(-1 * slow, Math.min(1 * slow, posX * slow));
            // posY = Math.max(-1 * slow, Math.min(1 * slow, posY * slow));

        posX = Math.max(-1, Math.min(1, posX));// * deadbandRatio;
        posY = Math.max(-1, Math.min(1, posY));// * deadbandRatio;
        
        SmartDashboard.putNumber("posX", posX);
        SmartDashboard.putNumber("posY", posY);
        // SmartDashboard.putNumber("addedFRIC", addedFRIC);
    }

    public void setPrecision(boolean precision) {
        this.preciece = precision;
    }

    //Overrides all driving
    public void setStrafeLeft(boolean left) {
        //false == right
        this.strafe = true;
        this.strafeLeft = left;
    }

    public void setStrafe(boolean strafe) {
        this.strafe = strafe;
    }

    public void setElevatorUp(boolean up) {
        this.elevatorUp = up;
    }

    public boolean getPrecision() {
        return this.preciece;
    }

    public boolean getStrafe() {
        return this.strafe;
    }

    public boolean getStrafeLeft() {
        return this.strafeLeft;
    }

    public double getDriverLeftX() {
        return driver0.getLeftX();
    }

    public double getDriverLeftY() {
        return driver0.getLeftY();
    }

    public double getDriverRightX() {
        return driver0.getRightX();
    }

    public double getRightPosX() {
        return rightPosX;
    }

    public double getPosX() {
        return posX;
    }

    public double getPosY() {
        return posY;
    }

    public double getPosR() {
        return posR;
    }

}