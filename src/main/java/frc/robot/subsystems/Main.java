package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

import frc.robot.commands.SwerveTeleop;

public class Main {
    public static void main(String[] args) {
        System.out.println("Hello, world!");
        AprilTagFieldLayout SIGMA = Constants.PhotonConsts.FIELD_LAYOUT;
        for(int i = 1; i < 23; i++) {
            Pose3d beta = SIGMA.getTagPose(i).get();
            System.out.println("TAG IG " +  i);
            System.out.println("TAG X " +  beta.getX());
            System.out.println("TAG Y " +  beta.getY());
            System.out.println("TAG R " +  beta.getRotation().toRotation2d().getDegrees());
            System.out.println();
         }

         
    }
}


// public  class test {
//     private AprilTagFieldLayout SIGMA = Constants.PhotonConsts.FIELD_LAYOUT;

//     public test() {

//     }

//     public void printAllLayouts() {
//         for(int i = 1; i < 23; i++) {
//             Pose3d beta = SIGMA.getTagPose(i).get();
//             System.out.println("TAG IG " +  i);
//             System.out.println("TAG X " +  beta.getX());
//             System.out.println("TAG Y " +  beta.getY());
//             System.out.println("TAG R " +  beta.getRotation().toRotation2d().getDegrees());
//         }
//     }
// }


