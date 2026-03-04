package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.geometry.Pose;

public final class FieldConstants {

    private FieldConstants() {}

    // ====== GOAL POSES ======
    public static final Pose BLUE_GOAL = new Pose(4, 413);
    public static final Pose RED_GOAL  = new Pose(140, 143);

    // ====== CORNER RESET POSES ======
    // Used for manual OPTIONS reset (no LL required)

    public static final Pose BLUE_CORNER_RESET =
            new Pose(
                    135,                     // X (inches)
                    9,                       // Y (inches)
                    Math.toRadians(90)       // Heading in radians
            );

    public static final Pose RED_CORNER_RESET =
            new Pose(
                    9,                     // X (inches)
                    9,                       // Y (inches)
                    Math.toRadians(90)       // Heading in radians
            );

}