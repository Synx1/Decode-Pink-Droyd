package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.geometry.Pose;

public class RobotState {

    // Default pose if auto never ran
    public static Pose startPose = new Pose(0, 0, 0);

    // Optional helper
    public static void setFromPose(Pose pose) {
        if (pose != null) {
            startPose = pose;
        }
    }
}