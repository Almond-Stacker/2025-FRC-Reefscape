package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConsts;

public class ControllerSubsystem extends SubsystemBase{
    CommandXboxController driver0;

    double accX;
    double posX; // controller value

    double accY;
    double posY;

    public ControllerSubsystem(CommandXboxController driver0) {
        this.driver0 = driver0;

        accX = 0;
        posX = 0;
        accY = 0;
        posY = 0;
    }

    @Override
    public void periodic() {
        accX = getDriverLeftX();
        accY = getDriverLeftY();

        double diffX = accX - posX;
        double diffY = accY - posY;

        if(accX >= Math.abs(posX)) {
            posX = accX;
        }
        else {
            posX += (accX - posX) * ControllerConsts.FRIC;
        }

        if(accY >= Math.abs(posY)) {
            posY = accY;
        }
        else {
            posY += (accY - posY) * ControllerConsts.FRIC;
        }
        
        

        posX = Math.max(-1, Math.min(1, posX));
        posY = Math.max(-1, Math.min(1, posY));
    }

    public double getDriverLeftX() {
        return driver0.getLeftX();
    }

    public double getDriverLeftY() {
        return driver0.getLeftY();
    }

    public double getPosX() {
        return posX;
    }

    public double getPosY() {
        return posY;
    }

}
