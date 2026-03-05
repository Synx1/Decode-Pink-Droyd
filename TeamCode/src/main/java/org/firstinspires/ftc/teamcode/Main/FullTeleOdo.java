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
import org.firstinspires.ftc.teamcode.SubSystem.MathUtilities;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.SubSystem.Turret;

@TeleOp(name = "FullTeleOdo", group = "Main")
@Config
public class FullTeleOdo extends OpMode {

    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;

    private boolean isBlue = true;
    private Pose goalPose;

    private boolean lastOptionsPressed      = false;
    private boolean lastRightTriggerPressed = false;

    // D-pad edge detection for shooter presets
    private boolean lastDpadUp   = false;
    private boolean lastDpadDown = false;

    // Loop timing
    private double lastLoopTimestamp = 0;
    private double lastLoopTime      = 0;

    // ============================
    // Shooter presets (ticks/sec)
    // ============================
    public static double PRESET_NEAR = 1300; // tune in Dashboard
    public static double PRESET_FAR  = 1550; // tune in Dashboard

    private boolean usingFar = false; // false = NEAR, true = FAR

    @Override
    public void init() {
        // ============================
        // PHOTON SETUP (DO THIS FIRST)
        // ============================
        // IMPORTANT: Photon requires hubs connected via USB (NOT RS485).
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Keep true ONLY if your servo power is Photon-compatible (direct hub ports / goBILDA injector).
        // If using REV Servo Hub / external non-USB servo power device, set false.
        PhotonCore.PARALLELIZE_SERVOS = true;

        // Optional tuning (docs recommend 8; raising too high can cause issues)
        PhotonCore.experimental.setMaximumParallelCommands(8);

        PhotonCore.enable();
        // Requested start pose
        Pose startPose = new Pose(64.0, 8.0, Math.toRadians(90));

        driveTrain = new DriveTrain(hardwareMap, startPose);
        shooter    = new Shooter(hardwareMap);
        intake     = new Intake(hardwareMap);
        turret     = new Turret(hardwareMap);

        isBlue   = true;
        goalPose = BLUE_GOAL;

        shooter.off();
        intake.spinOff();
        turret.on();
        turret.resetTurret(); // Set home to 0°

        // Default shooter preset: NEAR
        usingFar = false;
        shooter.setTarget(PRESET_NEAR);

        telemetry.addLine("Init: D-Pad LEFT = BLUE, RIGHT = RED");
        telemetry.addLine("L2 = odo auto-aim turret | turret goes home when not aiming");
        telemetry.addLine("D-Pad UP = FAR preset, DOWN = NEAR preset");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        Gamepad gp = gamepad1;

        // Alliance select
        if (gp.dpad_left)  isBlue = true;
        if (gp.dpad_right) isBlue = false;

        goalPose = isBlue ? BLUE_GOAL : RED_GOAL;

        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
        telemetry.addData("Goal", "X %.1f  Y %.1f", goalPose.getX(), goalPose.getY());
        telemetry.update();
    }

    @Override
    public void start() {
        driveTrain.startTeleOp();
        lastLoopTimestamp = getRuntime();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        lastLoopTime = now - lastLoopTimestamp;
        lastLoopTimestamp = now;

        Gamepad gp = gamepad1;

        // ===== DRIVE (ODO) =====
        driveTrain.periodic();
        driveTrain.drive(gp);

        Pose robotPose = driveTrain.getPose(); // heading in radians

        // ===== INTAKE =====
        if (gp.right_bumper)      intake.spinIn();
        else if (gp.left_bumper)  intake.spinOut();
        else                      intake.spinOff();

        // ===== TURRET (ODO AUTO-AIM OR HOME) =====
        boolean autoAim = gp.left_trigger > 0.5;

        if (autoAim) {
            // Auto-aim to goal using odometry
            turret.automatic();
            turret.face(goalPose, robotPose); // pure odo aiming
        } else {
            // Return to home (0°) when not aiming
            turret.automatic();
            turret.resetTurret(); // Sets target to 0°
        }

        turret.periodic();

        // ===== SHOOTER PRESETS (NEAR / FAR) =====
        boolean dpadUp   = gp.dpad_up;
        boolean dpadDown = gp.dpad_down;

        // D-pad UP -> FAR
        if (dpadUp && !lastDpadUp) {
            usingFar = true;
            shooter.setTarget(PRESET_FAR);
        }
        // D-pad DOWN -> NEAR
        if (dpadDown && !lastDpadDown) {
            usingFar = false;
            shooter.setTarget(PRESET_NEAR);
        }

        lastDpadUp   = dpadUp;
        lastDpadDown = dpadDown;

        // Shooter on/off with R2 (edge-detected)
        boolean rightTriggerPressed = gp.right_trigger > 0.5;
        if (rightTriggerPressed && !lastRightTriggerPressed) {
            shooter.shooterToggle();
        }
        lastRightTriggerPressed = rightTriggerPressed;

        shooter.periodic();

        // ===== CLAW =====
        if (gp.x) shooter.clawOpen();
        else      shooter.clawClose();

        // ===== CORNER RESET (OPTIONS) =====
        boolean optionsPressed = gp.options;
        if (optionsPressed && !lastOptionsPressed) {
            Pose cornerPose = isBlue ? BLUE_CORNER_RESET : RED_CORNER_RESET;
            driveTrain.getFollower().setPose(cornerPose);
            robotPose = cornerPose;
            telemetry.addLine(isBlue ? "Corner reset: BLUE" : "Corner reset: RED");
        }
        lastOptionsPressed = optionsPressed;

        // ===== TELEMETRY =====
        double distanceToGoal = MathUtilities.distance(robotPose, goalPose);

        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
        telemetry.addData("Robot", "X %.1f  Y %.1f  H %.2f rad",
                robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        telemetry.addData("Goal", "X %.1f  Y %.1f", goalPose.getX(), goalPose.getY());

        telemetry.addData("Dist(in)", "%.1f", distanceToGoal);

        telemetry.addData("Turret", turret.getTelemetryString());
        telemetry.addData("Turret Auto(L2)", autoAim);

        telemetry.addData("Shooter Preset", usingFar ? "FAR" : "NEAR");
        telemetry.addData("Shooter Target", shooter.getTarget());
        telemetry.addData("Shooter Vel", shooter.getVelocity());
        telemetry.addData("Shooter AtSpeed", shooter.atTarget());

        telemetry.addData("Loop ms", "%.2f", lastLoopTime * 1000.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.off();
        intake.spinOff();
        turret.off();
    }
}