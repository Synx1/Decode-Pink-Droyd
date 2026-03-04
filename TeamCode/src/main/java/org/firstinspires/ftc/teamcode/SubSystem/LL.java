package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
public class LL {

    private final Limelight3A limelight;

    // --- Stored / processed vision pose (Pedro coords) ---
    private Pose lastPedroPose = null;
    private long lastUpdateTimeMs = 0L;

    // --- Tunables (Dashboard) ---

    /** 0.0 = trust odometry only, 1.0 = snap fully to vision */
    public static double blendAlpha = 0.35;

    /** Vision data older than this (ms) is ignored */
    public static long maxPoseAgeMs = 300;

    /** Ignore vision if its position is this far away from odometry (inches) */
    public static double maxTranslationJumpInches = 12.0;

    /** Ignore vision if heading jump is bigger than this (radians) */
    public static double maxHeadingJumpRad = Math.toRadians(25.0);

    public LL(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // MegaTag1
        limelight.start();
    }

    /**
     * Reads Limelight, converts to Pedro coordinates, stores it, and returns it.
     * Returns null if no valid vision.
     */
    public Pose getVisionPose() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid())
            return null;

        Pose3D botpose = result.getBotpose();
        if (botpose == null)
            return null;

        Position pos = botpose.getPosition().toUnit(DistanceUnit.INCH);

        Pose2D pose2D = new Pose2D(
                DistanceUnit.INCH,
                pos.x,
                pos.y,
                AngleUnit.DEGREES,
                botpose.getOrientation().getYaw()
        );

        // FTC / Decode coords -> Pedro pose
        Pose ftcPose =
                PoseConverter.pose2DToPose(pose2D, InvertedFTCCoordinates.INSTANCE);

        Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        // store for blending
        lastPedroPose = pedroPose;
        lastUpdateTimeMs = System.currentTimeMillis();

        return pedroPose;
    }

    /**
     * True if we have a recent Pedro pose from vision.
     */
    public boolean hasFreshPose() {
        if (lastPedroPose == null) return false;
        long age = System.currentTimeMillis() - lastUpdateTimeMs;
        return age <= maxPoseAgeMs;
    }

    /**
     * Last Pedro pose directly from vision (no blending).
     * May be null if no vision yet.
     */
    public Pose getLastPedroPose() {
        return lastPedroPose;
    }

    /**
     * Blend odometry pose toward the last vision pose.
     * If no fresh / sane vision, returns odoPose unchanged.
     */
    public Pose getBlendedPose(Pose odoPose) {
        if (!hasFreshPose() || odoPose == null || lastPedroPose == null) {
            return odoPose;
        }

        // Check translation jump
        double dx = lastPedroPose.getX() - odoPose.getX();
        double dy = lastPedroPose.getY() - odoPose.getY();
        double distJump = Math.hypot(dx, dy);

        if (distJump > maxTranslationJumpInches) {
            // Too far off, ignore vision
            return odoPose;
        }

        // Check heading jump
        double odoHeading = odoPose.getHeading();
        double visHeading = lastPedroPose.getHeading();
        double headingDiff = angleWrap(visHeading - odoHeading);

        if (Math.abs(headingDiff) > maxHeadingJumpRad) {
            // Vision heading is suspicious, ignore
            return odoPose;
        }

        double alpha = clamp(blendAlpha, 0.0, 1.0);

        // Blend x, y
        double x = lerp(odoPose.getX(), lastPedroPose.getX(), alpha);
        double y = lerp(odoPose.getY(), lastPedroPose.getY(), alpha);

        // Blend heading along shortest path
        double heading = angleWrap(odoHeading + headingDiff * alpha);

        return new Pose(x, y, heading);
    }

    // --- Helpers ---

    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }


    private double angleWrap(double angle) {
        while (angle >= Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}