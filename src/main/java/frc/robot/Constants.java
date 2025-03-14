package frc.robot;

public class Constants {
    public static final int BEAMBREAK_ID = 0; 
    
    public static final class PrimaryElevatorConstants {
        public static final int LEFT_ELEVATOR_MOTOR_ID = 14;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 15;
        public static final int ENCODER_ID = 1;

        public static final double KP = 0.3;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
    }

    public static final class InnerElevatorConstants {
        public static final int MOTOR_ID = 22;

        public static final double KP = 0.25;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double KS = 0;
        public static final double KG = 0.01;
        public static final double KV = 0;

        public static final double GRAVITY_NEGATION = 0.2;
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 16;
        public static final int INDEX_MOTOR_ID = 25;
        public static final int ENCODER_ID = 0;

        public static final double KP = 0.012;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double KS = 0;
        public static final double KG = 0.003;
        public static final double KV = 0;
    }

    public static final class ClimbConstants {
        public static final int MOTOR_ID = 27;
        public static final int ENCODER_ID = 1;

        //need to get later
        public static final double MAX_THRESHOLD = 270;
        public static final double MIN_THRESHOLD = 0;

    }

    public static final class ControllerConsts {
        public static final double FRIC = 0.07;
        public static final double PRECIECE_ADD_FRIC = 0.7; //how much acceleration will slow when in preciece mode
        public static final double PRECIECE_ADD_TRANSLATIONAL_DEADBAND_RATIO = 0.4; // how much slower the max speed goes when in preciece mode
        public static final double PRECIECE_ADD_ROTATIONAL_DEADBAND_RATIO = 0.5; // how much slower the max rotational speed goes when in preciece mode
        public static final double DEADBAND_RATIO = 0.6;//absolute max speed cutoff when elevator up

        public static final double SLOW_RATIO = 0.4;//how fast it will move when elevator up
        public static final double STRAFE_RATIO = 0.1;
        
    }

}
