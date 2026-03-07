package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class LLtrack {

    // --- Hardware ---
    private final DcMotorEx turret;
    private final Limelight3A limelight;

    // --- Tag IDs ---
    public static int BLUE_TAG_ID = 20;
    public static int RED_TAG_ID  = 24;

    // --- Alliance flag ---
    private boolean isBlue = true;

    // --- Home position (encoder ticks) ---
    // Captured once in constructor during init, then locked forever.
    private double homePositionTicks;
    private boolean homeLocked = false;

    // --- Limelight tx PIDF (DEGREES error) ---
    public static double llKp = 0.015;
    public static double llKi = 0.0;
    public static double llKd = 0.0001;
    public static double llKf = 0.0;

    public static double llToleranceDeg = 1.5;
    public static double maxPower = 0.7;

    // --- Home P control (encoder ticks) ---
    // Used both to return to home AND to actively hold it once there.
    public static double homeKp = 0.003;
    public static double homeMaxPower = 0.5;
    public static double homeTolerance = 20.0;

    // --- Limelight PID state ---
    private double llIntegral  = 0.0;
    private double llLastError = 0.0;

    public LLtrack(HardwareMap hardwareMap, boolean startBlue) {
        turret = hardwareMap.get(DcMotorEx.class, "TT");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);

        limelight.start();

        isBlue = startBlue;

        // ── Lock home at init position, never changes during auto ──
        homePositionTicks = turret.getCurrentPosition();
        homeLocked = true;
    }

    // =========================================================
    // Public API
    // =========================================================

    public void setAlliance(boolean blue) {
        isBlue = blue;
    }

    /** No-op once locked. Won't accidentally overwrite init home. */
    public void saveHome() {
        if (!homeLocked) {
            homePositionTicks = turret.getCurrentPosition();
        }
    }

    public void unlockHome() {
        homeLocked = false;
    }

    /**
     * Main update.
     *
     * trackingButtonHeld == true  → turret is FREE to move, LL PID drives it to tag
     * trackingButtonHeld == false → turret is LOCKED to home, P-control holds it there
     *                               (actively resists any disturbance, not just coasting)
     *
     * @return true if aligned to tag (only meaningful when tracking)
     */
    public boolean update(boolean trackingButtonHeld, double dtSeconds) {
        if (trackingButtonHeld) {
            return alignWithTag(dtSeconds);
        } else {
            resetLimelightPID();
            holdHome();          // actively lock position at home every loop
            return false;
        }
    }

    public boolean isAtHome() {
        return Math.abs(homePositionTicks - turret.getCurrentPosition()) <= homeTolerance;
    }

    public Double getAllianceTx() {
        return getTxForAllianceTag();
    }

    public double getHomePositionTicks() { return homePositionTicks; }
    public int getCurrentTicks()         { return turret.getCurrentPosition(); }

    // =========================================================
    // Core logic
    // =========================================================

    private boolean alignWithTag(double dtSeconds) {
        Double tx = getTxForAllianceTag();
        if (tx == null) {
            // No tag visible — snap back to home while waiting
            holdHome();
            resetLimelightPID();
            return false;
        }

        double error      = tx;
        llIntegral       += error * dtSeconds;
        double derivative = (dtSeconds > 0) ? (error - llLastError) / dtSeconds : 0.0;
        llLastError = error;

        double output = llKp * error
                + llKi * llIntegral
                + llKd * derivative
                + Math.signum(error) * llKf;

        output = clamp(output, -maxPower, maxPower);
        turret.setPower(output);

        return Math.abs(error) <= llToleranceDeg;
    }

    /**
     * Actively holds turret at home using P control every loop.
     * Motor is never left floating — always commanded toward home.
     */
    private void holdHome() {
        double errorTicks = homePositionTicks - turret.getCurrentPosition();
        double power      = clamp(homeKp * errorTicks, -homeMaxPower, homeMaxPower);
        turret.setPower(power);
    }

    private Double getTxForAllianceTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        int desiredId = isBlue ? BLUE_TAG_ID : RED_TAG_ID;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f != null && f.getFiducialId() == desiredId) {
                return f.getTargetXDegrees();
            }
        }
        return null;
    }

    private void resetLimelightPID() {
        llIntegral  = 0.0;
        llLastError = 0.0;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}