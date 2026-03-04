package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.geometry.Pose;
public final class MathUtilities {

    private MathUtilities() { }
    //math idk
    public static double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // black ness + kenson = she gon call me babyboo = ABA tojifeetlicker combo
    public static double distance(Pose robotPose, Pose targetPose) {
        return distance(
                robotPose.getX(),
                robotPose.getY(),
                targetPose.getX(),
                targetPose.getY()
        );
    }
}