package org.firstinspires.ftc.teamcode.Main;

import static org.firstinspires.ftc.teamcode.SubSystem.FieldConstants.BLUE_CORNER_RESET;
import static org.firstinspires.ftc.teamcode.SubSystem.FieldConstants.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.SubSystem.FieldConstants.RED_CORNER_RESET;
import static org.firstinspires.ftc.teamcode.SubSystem.FieldConstants.RED_GOAL;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.teamcode.SubSystem.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.SubSystem.LLtrack;
import org.firstinspires.ftc.teamcode.SubSystem.MathUtilities;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;

@TeleOp(name = "FullTeleSimple", group = "Main")
@Config
public class FullTeleSimple extends OpMode {

    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private LLtrack llTrack;

    // Alliance / goal selection
    private boolean isBlue = true;
    private Pose goalPose;

    // Edge-detection flags
    private boolean lastOptionsPressed = false;
    private boolean lastRightTriggerPressed = false;

    // D-pad edge detection (for presets during OpMode)
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // Loop time tracking
    private double lastLoopTimestamp = 0;
    private double lastLoopTime = 0;

    // ============================
    // Shooter Presets (ticks/sec)
    // ============================
    public static double PRESET_NEAR    = 1150;
    public static double PRESET_MIDNEAR = 1350;
    public static double PRESET_FARNEAR = 1400;
    public static double PRESET_FAR     = 1450;
    public static double PRESET_FARFAR  = 1500;

    private int presetIndex = 0; // 0=NEAR, 1=MIDNEAR, 2=FARNEAR, 3=FAR, 4=FARFAR

    private double getPresetValue(int idx) {
        switch (idx) {
            case 0: return PRESET_NEAR;
            case 1: return PRESET_MIDNEAR;
            case 2: return PRESET_FARNEAR;
            case 3: return PRESET_FAR;
            case 4:
            default:
                return PRESET_FARFAR;
        }
    }

    private String getPresetName(int idx) {
        switch (idx) {
            case 0: return "NEAR";
            case 1: return "MIDNEAR";
            case 2: return "FARNEAR";
            case 3: return "FAR";
            case 4:
            default:
                return "FARFAR";
        }
    }

    @Override
    public void init() {
        // ============================
        // PHOTON SETUP (DO THIS FIRST)
        // ============================
        // IMPORTANT: Photon requires hubs connected via USB (NOT RS485).
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); //k
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Keep true ONLY if your servo power is Photon-compatible (direct hub ports / goBILDA injector).
        // If using REV Servo Hub / external non-USB servo power device, set false.
        PhotonCore.PARALLELIZE_SERVOS = true;

        // Optional tuning (docs recommend 8; raising too high can cause issues)
        PhotonCore.experimental.setMaximumParallelCommands(8);

        PhotonCore.enable();

        // ============================
        // SUBSYSTEMS
        // ============================
        Pose startPose = new Pose(64.0, 8.0, Math.toRadians(90));

        driveTrain = new DriveTrain(hardwareMap, startPose);
        shooter    = new Shooter(hardwareMap);
        intake     = new Intake(hardwareMap);
        llTrack    = new LLtrack(hardwareMap, true); // start as blue

        isBlue   = true;
        goalPose = BLUE_GOAL;

        // default preset
        presetIndex = 0;
        shooter.setTarget(getPresetValue(presetIndex));

        shooter.off();
        intake.spinOff();

        telemetry.addLine("Init: D-Pad LEFT = BLUE, RIGHT = RED");
        telemetry.addLine("During OpMode: D-Pad LEFT/RIGHT cycles shooter presets");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Clear caches every loop to avoid stale reads in MANUAL caching
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();

        Gamepad gp = gamepad1;

        // Alliance selection before START
        if (gp.dpad_left)  isBlue = true;
        if (gp.dpad_right) isBlue = false;

        // Update goal pose and LLtrack alliance
        goalPose = isBlue ? BLUE_GOAL : RED_GOAL;
        llTrack.setAlliance(isBlue);

        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
        telemetry.addData("Goal", "X: %.1f, Y: %.1f", goalPose.getX(), goalPose.getY());
        telemetry.addData("Turret Yaw (deg)", "%.1f", Math.toDegrees(llTrack.getCurrentYaw()));
        telemetry.update();
    }

    @Override
    public void start() {
        driveTrain.startTeleOp();
        lastLoopTimestamp = getRuntime();
    }

    @Override
    public void loop() {
        // Clear caches FIRST in every single run loop (MANUAL bulk caching rule)
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();

        updateLoopTime();
        Gamepad gp = gamepad1;

        // ====== DRIVETRAIN ======
        driveTrain.periodic();
        driveTrain.drive(gp);

        Pose robotPose = driveTrain.getPose();

        // ====== INTAKE ======
        if (gp.right_bumper) {
            intake.spinIn();
        } else if (gp.left_bumper) {
            intake.spinOut();
        } else {
            intake.spinOff();
        }

        // ====== TURRET / LL TRACKING ======
        boolean tracking = gp.left_trigger > 0.5;
        boolean turretAligned = llTrack.update(tracking, lastLoopTime);

        // ====== SHOOTER PRESET CONTROL ======
        boolean dpadLeft = gp.dpad_left;
        boolean dpadRight = gp.dpad_right;

        if (dpadRight && !lastDpadRight) {
            presetIndex++;
            if (presetIndex > 4) presetIndex = 0;
            shooter.setTarget(getPresetValue(presetIndex));
        }
        if (dpadLeft && !lastDpadLeft) {
            presetIndex--;
            if (presetIndex < 0) presetIndex = 4;
            shooter.setTarget(getPresetValue(presetIndex));
        }

        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;

        // R2 toggles shooter on/off (edge-detected)
        boolean rightTriggerPressed = gp.right_trigger > 0.5;
        if (rightTriggerPressed && !lastRightTriggerPressed) {
            shooter.shooterToggle();
        }
        lastRightTriggerPressed = rightTriggerPressed;

        shooter.periodic();

        // ====== CLAW CONTROL ======
        if (gp.x) {
            shooter.clawOpen();
        } else {
            shooter.clawClose();
        }

        // ====== CORNER RESET (OPTIONS) ======
        boolean optionsPressed = gp.options;
        if (optionsPressed && !lastOptionsPressed) {
            Pose cornerPose = isBlue ? BLUE_CORNER_RESET : RED_CORNER_RESET;
            driveTrain.getFollower().setPose(cornerPose);
            robotPose = cornerPose;

            telemetry.addLine(isBlue ? "Corner reset: BLUE" : "Corner reset: RED");
        }
        lastOptionsPressed = optionsPressed;

        // ====== TELEMETRY ======
        double distanceToGoal = MathUtilities.distance(robotPose, goalPose);

        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
        telemetry.addData("Goal", "X: %.1f, Y: %.1f", goalPose.getX(), goalPose.getY());
        telemetry.addData("Robot Pose", "X: %.1f, Y: %.1f, H: %.2f",
                robotPose.getX(), robotPose.getY(), robotPose.getHeading());

        telemetry.addData("Distance to Goal (in)", "%.1f", distanceToGoal);

        telemetry.addData("Shooter Preset", "%s (%d/5)", getPresetName(presetIndex), (presetIndex + 1));
        telemetry.addData("Shooter Target (t/s)", "%.0f", getPresetValue(presetIndex));
        telemetry.addData("Shooter Velocity (t/s)", "%.0f", shooter.getVelocity());

        telemetry.addData("Turret Yaw (deg)", "%.1f", Math.toDegrees(llTrack.getCurrentYaw()));
        telemetry.addData("Turret Last Good (deg)", "%.1f", Math.toDegrees(llTrack.getLastGoodPositionRadians()));
        telemetry.addData("Turret Tracking (L2)", tracking);
        telemetry.addData("Turret Aligned", turretAligned);

        Double allianceTx = llTrack.getAllianceTx();
        if (allianceTx != null) {
            telemetry.addData("Alliance Tag Tx (deg)", "%.1f", allianceTx);
        } else {
            telemetry.addData("Alliance Tag Tx (deg)", "NO TAG");
        }

        telemetry.addData("Loop Time (ms)", "%.3f", lastLoopTime * 1000.0);
        telemetry.addData("Loop Hz", "%.1f", (lastLoopTime > 0) ? 1.0 / lastLoopTime : 0.0);

        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.off();
        intake.spinOff();
    }

    // ============================
    // Helpers
    // ============================
    private void updateLoopTime() {
        double now = getRuntime();
        lastLoopTime = now - lastLoopTimestamp;
        lastLoopTimestamp = now;
    }
}