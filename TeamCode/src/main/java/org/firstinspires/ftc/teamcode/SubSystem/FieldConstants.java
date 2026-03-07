package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.geometry.Pose;

public final class FieldConstants {

    private FieldConstants() {}

    // ====== GOAL POSES ======
    public static final Pose BLUE_GOAL = new Pose(4, 413);
    public static final Pose RED_GOAL  = new Pose(140, 143);

    // ====== CORNER RESET POSES ======
    public static final Pose BLUE_CORNER_RESET =
            new Pose(
                    132,
                    11,
                    Math.toRadians(90)
            );

    public static final Pose RED_CORNER_RESET =
            new Pose(
                    9,
                    9,
                    Math.toRadians(90)
            );
}