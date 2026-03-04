package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class LLtrack {

    // --- Hardware ---
    private final Turret turret;
    private final Limelight3A limelight;

    // --- Tag IDs ---
    public static int BLUE_TAG_ID = 20;
    public static int RED_TAG_ID  = 24;

    // --- Alliance flag ---
    private boolean isBlue = true;

    // --- Last good shooting position (radians) ---
    public static double preAimTolerance = Math.toRadians(2.0); // tolerance in radians
    private double lastGoodPositionRadians = 0.0;

    // --- Limelight tx PIDF (DEGREES error) ---
    public static double llKp = 0.0155;
    public static double llKi = 0.0;
    public static double llKd = 0.0;
    public static double llKf = 0.0; // optional feedforward bias

    // Acceptable aim error in degrees
    public static double llToleranceDeg = 1.0;

    // Max motor power for LL aiming
    public static double maxPower = 0.5;

    // --- Limelight PID state ---
    private double llIntegral = 0.0;
    private double llLastError = 0.0;

    // --- Internal state for "LL just turned on" edge detection ---
    private boolean wasTracking = false;
    private boolean preAimActive = false;

    public LLtrack(HardwareMap hardwareMap, boolean startBlue) {
        // Initialize turret subsystem
        turret = new Turret(hardwareMap);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // your MegaTag or tracking pipeline
        limelight.start();

        isBlue = startBlue;

        // Initialize last good shot position to home (0)
        lastGoodPositionRadians = 0.0;

        // Set turret to home position
        turret.resetTurret();
    }

    // =========================================================
    // Public API
    // =========================================================

    /** Change alliance on the fly (call from init_loop with dpad). */
    public void setAlliance(boolean blue) {
        isBlue = blue;
    }

    /**
     * Main update - must be called every loop:
     *  - If trackingButtonHeld == false: turret goes to home (0°) using automatic control
     *  - If trackingButtonHeld == true:
     *      * On the first frame (rising edge), pre-aim to lastGoodPositionRadians
     *      * Once close enough, switch to Limelight tx manual control for fine alignment
     *
     * @param trackingButtonHeld  L2 state (true when pressed)
     * @param dtSeconds           loop time in seconds
     * @return true if we are aligned to tag (|tx| <= tolerance) via Limelight, false otherwise
     */
    public boolean update(boolean trackingButtonHeld, double dtSeconds) {
        // Always call turret periodic first
        turret.periodic();

        if (trackingButtonHeld) {
            // Rising edge: LL just turned on
            if (!wasTracking) {
                preAimActive = true;
            }
            wasTracking = true;

            // Phase 1: snap turret to last known good shot position
            if (preAimActive) {
                boolean atLastGood = goToLastGoodPosition();
                if (atLastGood) {
                    preAimActive = false;
                    // clear integral so LL PID starts clean
                    resetLimelightPID();
                }
                // While pre-aiming, don't report "locked" yet
                return false;
            }

            // Phase 2: normal Limelight alignment (manual control)
            return alignWithTag(dtSeconds);

        } else {
            // LL off: go home using automatic turret control
            wasTracking = false;
            preAimActive = false;
            resetLimelightPID();
            goHome();
            return false;
        }
    }

    /** For telemetry: current tx for the alliance tag, or null if none. */
    public Double getAllianceTx() {
        return getTxForAllianceTag();
    }

    public double getCurrentYaw() {
        return turret.getYaw();
    }

    public double getLastGoodPositionRadians() {
        return lastGoodPositionRadians;
    }

    // =========================================================
    // Core logic
    // =========================================================

    /**
     * Pre-aim: drive turret back to lastGoodPositionRadians using automatic control.
     * @return true if we're within preAimTolerance of lastGoodPositionRadians.
     */
    private boolean goToLastGoodPosition() {
        turret.automatic();
        turret.setYaw(lastGoodPositionRadians);

        double error = Math.abs(lastGoodPositionRadians - turret.getYaw());
        return error <= preAimTolerance;
    }

    /** Align turret so that Limelight tx → 0 for alliance-specific tag ID using manual control. */
    private boolean alignWithTag(double dtSeconds) {
        Double tx = getTxForAllianceTag();
        if (tx == null) {
            // No valid tag: stop turret and clear PID state
            turret.manual(0);
            resetLimelightPID();
            return false;
        }

        // PIDF on tx (degrees, want 0°)
        double error = tx;
        llIntegral += error * dtSeconds;
        double derivative = (dtSeconds > 0) ? (error - llLastError) / dtSeconds : 0.0;
        llLastError = error;

        double output = llKp * error
                + llKi * llIntegral
                + llKd * derivative
                + Math.signum(error) * llKf;

        // Clamp power
        if (output >  maxPower) output =  maxPower;
        if (output < -maxPower) output = -maxPower;

        turret.manual(output);

        // If we're within tolerance, remember this as the new "good shot" yaw
        boolean withinTolerance = Math.abs(error) <= llToleranceDeg;
        if (withinTolerance) {
            lastGoodPositionRadians = turret.getYaw();
        }

        return withinTolerance;
    }

    /** Drive turret back to home (0°) using automatic control. */
    private void goHome() {
        turret.automatic();
        turret.resetTurret(); // Sets target to 0
    }

    /** Get tx (deg) for the alliance-specific tag ID, or null if not visible. */
    private Double getTxForAllianceTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        int desiredId = isBlue ? BLUE_TAG_ID : RED_TAG_ID;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f != null && f.getFiducialId() == desiredId) {
                return f.getTargetXDegrees(); // tx in degrees
            }
        }

        // No matching ID found
        return null;
    }

    private void resetLimelightPID() {
        llIntegral = 0.0;
        llLastError = 0.0;
    }
}