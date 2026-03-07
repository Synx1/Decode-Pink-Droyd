package org.firstinspires.ftc.teamcode.Main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SubSystem.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.SubSystem.MathUtilities;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.SubSystem.Turret;

import static org.firstinspires.ftc.teamcode.SubSystem.FieldConstants.*;

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

    private boolean lastDpadUp   = false;
    private boolean lastDpadDown = false;

    private double lastLoopTimestamp = 0;
    private double lastLoopTime      = 0;

    public static double PRESET_NEAR = 1300;
    public static double PRESET_FAR  = 1550;

    private boolean usingFar = false;

    @Override
    public void init() {
        // ── Match your actual start tile here ──
        Pose startPose = new Pose(64.0, 8.0, Math.toRadians(90));

        driveTrain = new DriveTrain(hardwareMap, startPose);
        shooter    = new Shooter(hardwareMap);
        intake     = new Intake(hardwareMap);
        turret     = new Turret(hardwareMap, driveTrain.getFollower());

        isBlue   = true;
        goalPose = BLUE_GOAL;

        shooter.off();
        intake.spinOff();
        turret.on();

        usingFar = false;
        shooter.setTarget(PRESET_NEAR);

        telemetry.addLine("Init: D-Pad LEFT = BLUE, RIGHT = RED");
        telemetry.addLine("L2 = odo auto-aim turret (release = return home)");
        telemetry.addLine("D-Pad UP = FAR preset, DOWN = NEAR preset");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        Gamepad gp = gamepad1;

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

        // ===== DRIVE =====
        driveTrain.periodic();
        driveTrain.drive(gp);

        // Get live odo pose AFTER follower.update() inside periodic()
        Pose robotPose = driveTrain.getPose();

        // ===== INTAKE =====
        if (gp.right_bumper)      intake.spinIn();
        else if (gp.left_bumper)  intake.spinOut();
        else                      intake.spinOff();

        // ===== TURRET AUTO-AIM or RETURN HOME =====
        // L2 held → face goal using live odo pose
        // L2 released → return to init home position
        boolean autoAim = gp.left_trigger > 0.5;
        if (autoAim) {
            turret.face(goalPose, robotPose); // explicit pose so it uses the freshest odo reading
        } else {
            turret.goHome();
        }
        turret.periodic();

        // ===== SHOOTER PRESETS =====
        boolean dpadUp   = gp.dpad_up;
        boolean dpadDown = gp.dpad_down;

        if (dpadUp && !lastDpadUp) {
            usingFar = true;
            shooter.setTarget(PRESET_FAR);
        }
        if (dpadDown && !lastDpadDown) {
            usingFar = false;
            shooter.setTarget(PRESET_NEAR);
        }

        lastDpadUp   = dpadUp;
        lastDpadDown = dpadDown;

        boolean rightTriggerPressed = gp.right_trigger > 0.5;
        if (rightTriggerPressed && !lastRightTriggerPressed) {
            shooter.shooterToggle();
        }
        lastRightTriggerPressed = rightTriggerPressed;

        shooter.periodic();

        // ===== CLAW =====
        if (gp.x) shooter.clawOpen();
        else      shooter.clawClose();

        // ===== CORNER RESET =====
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
        telemetry.addData("Robot", "X %.1f  Y %.1f  H %.2f°",
                robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Goal", "X %.1f  Y %.1f", goalPose.getX(), goalPose.getY());
        telemetry.addData("Dist(in)", "%.1f", distanceToGoal);

        telemetry.addData("Turret", turret.getTelemetryString());
        telemetry.addData("Turret Auto(L2)", autoAim);

        telemetry.addData("Shooter Preset", usingFar ? "FAR" : "NEAR");
        telemetry.addData("Shooter Target",  shooter.getTarget());
        telemetry.addData("Shooter Vel",     shooter.getVelocity());
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