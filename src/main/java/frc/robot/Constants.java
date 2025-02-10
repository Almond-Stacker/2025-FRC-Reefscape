package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

//if want to import with .* to avoid typing exact class dependances
//consider seperate constants files
public class Constants {
    
    public static final class PrimaryElevatorConsts {
        public static final int leftElevatorMotorID = 14;
        public static final int rightElevatorMotorID = 15;
        public static final int encoderID = 1;

        //temopory estamate, although resonable value?
        //smaller the better, however don't want occelations 
        public static final double PID_TOLERANCE = 10;
        public static final double kP = 0.02;
        public static final double kI = 0;
        public static final double kD = 0;
        
        //tune this as well too, could be very off ##note it's reading height
        public static final TrapezoidProfile.Constraints PROFILE = new TrapezoidProfile.Constraints(2, 4);
        
    }

    public static final class InnerElevatorConsts {
        public static final int elevatorMotorID = 22;

        public static final double kP = 0.38;
        public static final double kI = 0;
        public static final double kD = 0.03;

        public static final double kS = 0;
        public static final double kG = 0.03;
        public static final double kV = 0;

        public static final double gravityNegationConstant = 0.7;
        public static final TrapezoidProfile.Constraints PROFILE = new TrapezoidProfile.Constraints(2, 4);
    }

    public static final class IntakeArmConsts {
        public static final int armMotorID = 17;
        public static final int encoderID = 0;
        public static final int suckMotorID = 21;
        public static final double OUT_TIMEOUT = 1;//one second 

        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 1);
        public static final double kS = 0;
        public static final double kG = 0.003;
        public static final double kV = 0;
    }

    public static final class PhotonConsts {
        public static final String CAM0_NAME = "front_photon_camera";
        public static final String CAM1_NAME = "";//set constants 2/9
    }

}
