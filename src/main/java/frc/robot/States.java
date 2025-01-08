package frc.robot;

import frc.robot.Constants.Photon;

public class States {
    // allow for easy changing of photon vision targets 
    public enum PhotonStates {
        driveTag4(Photon.tag4.targetID ,Photon.tag4.distance0, Photon.tag4.angle0, Photon.tag4.tagHeight);

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

    public enum ElevatorStates {
        // allow for easy changing of elevator states 
        kBottom(0.0),
        kMiddle(0.5),
        kTop(1.0);

        private final double height;

        ElevatorStates(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }

    }
}
