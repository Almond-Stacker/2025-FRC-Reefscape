package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Photon;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.subsystems.PrimaryElevatorSubsystem;

public class States {
    // allow for easy changing of photon vision targets 
    public enum PhotonStates {
        driveTag4(Photon.tag4.targetID ,Photon.tag4.distance0, Photon.tag4.angle0, Photon.tag4.tagHeight),
        tag1(Photon.tag1.targetID, Photon.tag1.desiredDistance1, Photon.tag1.desiredAngle1, Photon.tag1.tagHeight);

        public final int id;
        public final double distance;
        public final double angle; 
        public final double height;

        PhotonStates(int id, double distance, double angle, double height) {
            this.id = id;
            this.distance = distance;
            this.angle = angle;
            this.height = height;
        }
    }

    public enum PrimaryElevatorStates {
        // allow for easy changing of elevator states 
        HOME(0.57),
        L1(26),
        L2(99.5),
        L3(99.5),
        PRE_INTAKE(80),
        INTAKE(52),
        MIN(0),
        MAX(101);

        public final double height;

        PrimaryElevatorStates(double height) {
            this.height = height;
        }

        PrimaryElevatorStates() {
            this.height = 0; 
        }
    }

    public enum InnerElevatorStates {
        // allow for easy changing of elevator states 
        HOME(0.01),
        L1(),
        L2(),
        L3(4.8),
        MIN(0),
        MAX(5);

        public final double height;

        InnerElevatorStates(double height) {
            this.height = height;
        }

        InnerElevatorStates() {
            this.height = 0; 
        }
    } 

    public enum ArmStates {
        // allow for easy changing of elevator states 
        HOME(60),
        INTAKE(60),
        STARTING_POSITION(240),
        L1(210),
        L2(210),
        L3(210),
        FEED_OUT(207),
        MIN(0),
        MAX(360);

        public final double angle;

        ArmStates(double angle) {
            this.angle = angle;
        }

        ArmStates() {
            this.angle = 0; 
        }
    } 

    public enum IndexStates {
        // allow for easy changing of elevator states 
        STOP(0),
        INTAKE(1),
        FEED_OUT(-1);

        public final double speed;

        IndexStates(double speed) {
            this.speed = speed;
        }

        IndexStates() {
            this.speed = 0; 
        }
    } 

    public enum AlgaeIntakeStates {
        // allow for easy changing of elevator states 
        STOP(),
        INTAKE(),
        FEED_OUT();

        public final double speed;

        AlgaeIntakeStates(double speed) {
            this.speed = speed;
        }

        AlgaeIntakeStates() {
            this.speed = 0; 
        }
    } 

    public enum ClimbStates {
        STOP(0),
        CLIMB(0.5),
        DROP(-0.5);

        public final double speed;

        ClimbStates(double speed) {
            this.speed = speed;
        }

        ClimbStates() {
            this.speed = 0;
        }
    }

    public enum ledStates {
        /*
        * Fixed Palette Pattern
        */
        RAINBOW_RAINBOW_PALETTE(-0.99),
        RAINBOW_PARTY_PALETTE(-0.97),
        RAINBOW_OCEAN_PALETTE(-0.95),
        RAINBOW_LAVA_PALETTE(-0.93),
        RAINBOW_FOREST_PALETTE(-0.91),
        RAINBOW_WITH_GLITTER(-0.89),
        CONFETTI(-0.87),
        SHOT_RED(-0.85),
        SHOT_BLUE(-0.83),
        SHOT_WHITE(-0.81),
        SINELON_RAINBOW_PALETTE(-0.79),
        SINELON_PARTY_PALETTE(-0.77),
        SINELON_OCEAN_PALETTE(-0.75),
        SINELON_LAVA_PALETTE(-0.73),
        SINELON_FOREST_PALETTE(-0.71),
        BEATS_PER_MINUTE_RAINBOW_PALETTE(-0.69),
        BEATS_PER_MINUTE_PARTY_PALETTE(-0.67),
        BEATS_PER_MINUTE_OCEAN_PALETTE(-0.65),
        BEATS_PER_MINUTE_LAVA_PALETTE(-0.63),
        BEATS_PER_MINUTE_FOREST_PALETTE(-0.61),
        FIRE_MEDIUM(-0.59),
        FIRE_LARGE(-0.57),
        TWINKLES_RAINBOW_PALETTE(-0.55),
        TWINKLES_PARTY_PALETTE(-0.53),
        TWINKLES_OCEAN_PALETTE(-0.51),
        TWINKLES_LAVA_PALETTE(-0.49),
        TWINKLES_FOREST_PALETTE(-0.47),
        COLOR_WAVES_RAINBOW_PALETTE(-0.45),
        COLOR_WAVES_PARTY_PALETTE(-0.43),
        COLOR_WAVES_OCEAN_PALETTE(-0.41),
        COLOR_WAVES_LAVA_PALETTE(-0.39),
        COLOR_WAVES_FOREST_PALETTE(-0.37),
        LARSON_SCANNER_RED(-0.35),
        LARSON_SCANNER_GRAY(-0.33),
        LIGHT_CHASE_RED(-0.31),
        LIGHT_CHASE_BLUE(-0.29),
        LIGHT_CHASE_GRAY(-0.27),
        HEARTBEAT_RED(-0.25),
        HEARTBEAT_BLUE(-0.23),
        HEARTBEAT_WHITE(-0.21),
        HEARTBEAT_GRAY(-0.19),
        BREATH_RED(-0.17),
        BREATH_BLUE(-0.15),
        BREATH_GRAY(-0.13),
        STROBE_RED(-0.11),
        STROBE_BLUE(-0.09),
        STROBE_GOLD(-0.07),
        STROBE_WHITE(-0.05),
        /*
        * CP1: Color 1 Pattern
        */
        CP1_END_TO_END_BLEND_TO_BLACK(-0.03),
        CP1_LARSON_SCANNER(-0.01),
        CP1_LIGHT_CHASE(+0.01),
        CP1_HEARTBEAT_SLOW(+0.03),
        CP1_HEARTBEAT_MEDIUM(+0.05),
        CP1_HEARTBEAT_FAST(+0.07),
        CP1_BREATH_SLOW(+0.09),
        CP1_BREATH_FAST(+0.11),
        CP1_SHOT(+0.13),
        CP1_STROBE(+0.15),
        /*
        * CP2: Color 2 Pattern
        */
        CP2_END_TO_END_BLEND_TO_BLACK(+0.17),
        CP2_LARSON_SCANNER(+0.19),
        CP2_LIGHT_CHASE(+0.21),
        CP2_HEARTBEAT_SLOW(+0.23),
        CP2_HEARTBEAT_MEDIUM(+0.25),
        CP2_HEARTBEAT_FAST(+0.27),
        CP2_BREATH_SLOW(+0.29),
        CP2_BREATH_FAST(+0.31),
        CP2_SHOT(+0.33),
        CP2_STROBE(+0.35),
        /*
        * CP1_2: Color 1 and 2 Pattern
        */
        CP1_2_SPARKLE_1_ON_2(+0.37),
        CP1_2_SPARKLE_2_ON_1(+0.39),
        CP1_2_COLOR_GRADIENT(+0.41),
        CP1_2_BEATS_PER_MINUTE(+0.43),
        CP1_2_END_TO_END_BLEND_1_TO_2(+0.45),
        CP1_2_END_TO_END_BLEND(+0.47),
        CP1_2_NO_BLENDING(+0.49),
        CP1_2_TWINKLES(+0.51),
        CP1_2_COLOR_WAVES(+0.53),
        CP1_2_SINELON(+0.55),
        /*
        * Solid color
        */
        HOT_PINK(+0.57),
        DARK_RED(+0.59),
        RED(+0.61),
        RED_ORANGE(+0.63),
        ORANGE(+0.65),
        GOLD(+0.67),
        YELLOW(+0.69),
        LAWN_GREEN(+0.71),
        LIME(+0.73),
        DARK_GREEN(+0.75),
        GREEN(+0.77),
        BLUE_GREEN(+0.79),
        AQUA(+0.81),
        SKY_BLUE(+0.83),
        DARK_BLUE(+0.85),
        BLUE(+0.87),
        BLUE_VIOLET(+0.89),
        VIOLET(+0.91),
        WHITE(+0.93),
        GRAY(+0.95),
        DARK_GRAY(+0.97),
        BLACK(+0.99),
        OFF(0);

        public final double value;

        private ledStates(double value) {
        this.value = value;
        }
    }
}
