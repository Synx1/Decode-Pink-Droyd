package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.geometry.Pose;

public final class MathUtilities {

    private MathUtilities() { }

    public static double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public static double distance(Pose robotPose, Pose targetPose) {
        return distance(
                robotPose.getX(),
                robotPose.getY(),
                targetPose.getX(),
                targetPose.getY()
        );
    }

    /**
     * Returns turret-relative yaw (radians) needed to face targetPose from robotPose.
     * Output is normalized to (-π, π].
     */
    public static double faceAngle(Pose robotPose, Pose targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        return Turret.normalizeAngle(Math.atan2(dy, dx) - robotPose.getHeading());
    }

    /** Max diagonal of a 144×144 field in inches. */
    public static final double MAX_FIELD_DISTANCE = Math.sqrt(144.0 * 144.0 + 144.0 * 144.0); // ≈ 203.6
}