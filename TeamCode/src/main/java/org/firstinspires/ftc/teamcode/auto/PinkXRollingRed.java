package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.SubSystem.LLtrack;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PinkXRollingRed", group = "Autonomous")
@Configurable
public class PinkXRollingRed extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private Shooter shooter;
    private Intake intake;
    private LLtrack llTrack;

    // ── State Machine ──────────────────────────────────────────────
    private enum State {
        // Shoot0 sequence
        SHOOT0_PATH,
        SHOOT0_TRACK,
        SHOOT0_DUMP,

        // Intake_A sequence
        INTAKE_A_PATH,
        INTAKE_A_STOP,

        // Shoot1 sequence
        SHOOT1_INTAKE_PRE,
        SHOOT1_PATH,
        SHOOT1_TRACK,
        SHOOT1_DUMP,

        // Intake_B sequence
        INTAKE_B_PATH,
        INTAKE_B_STOP,

        // Shoot2 sequence
        SHOOT2_INTAKE_PRE,
        SHOOT2_PATH,
        SHOOT2_TRACK,
        SHOOT2_DUMP,

        // Intake_C sequence
        INTAKE_C_PATH,
        INTAKE_C_STOP,

        // Shoot3 sequence
        SHOOT3_INTAKE_PRE,
        SHOOT3_PATH,
        SHOOT3_TRACK,
        SHOOT3_DUMP,

        // Intake_D sequence
        INTAKE_D_PATH,
        INTAKE_D_STOP,

        // Final Shoot
        SHOOT_FINAL_INTAKE_PRE,
        SHOOT_FINAL_PATH,
        SHOOT_FINAL_TRACK,
        SHOOT_FINAL_DUMP,

        PARK,
        DONE
    }

    private State state = State.SHOOT0_PATH;
    private long stateStartMs      = 0;
    private long lastLoopMs        = 0;
    private long shooterStableStart = 0;

    // ── Init ───────────────────────────────────────────────────────
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
        llTrack = new LLtrack(hardwareMap, true);

        paths = new Paths(follower);

        shooter.shootFar();
        shooter.clawClose();
        intake.spinIdle();
        llTrack.setAlliance(true);

        lastLoopMs   = System.currentTimeMillis();
        stateStartMs = lastLoopMs;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.followPath(paths.Shoot0);
        state        = State.SHOOT0_PATH;
        stateStartMs = System.currentTimeMillis();
        lastLoopMs   = stateStartMs;
    }

    // ── Main Loop ──────────────────────────────────────────────────
    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        double dt = (now - lastLoopMs) / 1000.0;
        lastLoopMs = now;

        shooter.periodic();
        follower.update();

        switch (state) {

            // ── SHOOT0 ─────────────────────────────────────────────
            case SHOOT0_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(true);
                    transition(State.SHOOT0_TRACK, now);
                }
                break;

            case SHOOT0_TRACK: {
                boolean locked = llTrack.update(true, dt);
                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT0_DUMP, now);
                }
                break;
            }

            case SHOOT0_DUMP:
                llTrack.update(true, dt);
                if (elapsed(now) >= 2000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.Intake_A);
                    intake.spinIn();
                    transition(State.INTAKE_A_PATH, now);
                }
                break;

            // ── INTAKE_A ───────────────────────────────────────────
            case INTAKE_A_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.INTAKE_A_STOP, now);
                }
                break;

            case INTAKE_A_STOP:
                intake.spinIn();
                transition(State.SHOOT1_INTAKE_PRE, now);
                break;

            // ── SHOOT1 ─────────────────────────────────────────────
            case SHOOT1_INTAKE_PRE:
                if (elapsed(now) >= 1500) {
                    intake.spinIdle();
                    follower.followPath(paths.Shoot1);
                    transition(State.SHOOT1_PATH, now);
                }
                break;

            case SHOOT1_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(true);
                    transition(State.SHOOT1_TRACK, now);
                }
                break;

            case SHOOT1_TRACK: {
                boolean locked = llTrack.update(true, dt);
                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT1_DUMP, now);
                }
                break;
            }

            case SHOOT1_DUMP:
                llTrack.update(true, dt);
                if (elapsed(now) >= 2000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.Intake_B);
                    intake.spinIn();
                    transition(State.INTAKE_B_PATH, now);
                }
                break;

            // ── INTAKE_B ───────────────────────────────────────────
            case INTAKE_B_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.INTAKE_B_STOP, now);
                }
                break;

            case INTAKE_B_STOP:
                intake.spinIn();
                transition(State.SHOOT2_INTAKE_PRE, now);
                break;

            // ── SHOOT2 ─────────────────────────────────────────────
            case SHOOT2_INTAKE_PRE:
                if (elapsed(now) >= 1500) {
                    intake.spinIdle();
                    follower.followPath(paths.Shoot2);
                    transition(State.SHOOT2_PATH, now);
                }
                break;

            case SHOOT2_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(true);
                    transition(State.SHOOT2_TRACK, now);
                }
                break;

            case SHOOT2_TRACK: {
                boolean locked = llTrack.update(true, dt);
                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT2_DUMP, now);
                }
                break;
            }

            case SHOOT2_DUMP:
                llTrack.update(true, dt);
                if (elapsed(now) >= 2000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.Intake_C);
                    intake.spinIn();
                    transition(State.INTAKE_C_PATH, now);
                }
                break;

            // ── INTAKE_C ───────────────────────────────────────────
            case INTAKE_C_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.INTAKE_C_STOP, now);
                }
                break;

            case INTAKE_C_STOP:
                intake.spinIn();
                transition(State.SHOOT3_INTAKE_PRE, now);
                break;

            // ── SHOOT3 ─────────────────────────────────────────────
            case SHOOT3_INTAKE_PRE:
                if (elapsed(now) >= 1500) {
                    intake.spinIdle();
                    follower.followPath(paths.Shoot3);
                    transition(State.SHOOT3_PATH, now);
                }
                break;

            case SHOOT3_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(true);
                    transition(State.SHOOT3_TRACK, now);
                }
                break;

            case SHOOT3_TRACK: {
                boolean locked = llTrack.update(true, dt);
                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT3_DUMP, now);
                }
                break;
            }

            case SHOOT3_DUMP:
                llTrack.update(true, dt);
                if (elapsed(now) >= 2000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.Intake_D);
                    intake.spinIn();
                    transition(State.INTAKE_D_PATH, now);
                }
                break;

            // ── INTAKE_D ───────────────────────────────────────────
            case INTAKE_D_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.INTAKE_D_STOP, now);
                }
                break;

            case INTAKE_D_STOP:
                intake.spinIn();
                transition(State.SHOOT_FINAL_INTAKE_PRE, now);
                break;

            // ── FINAL SHOOT ────────────────────────────────────────
            case SHOOT_FINAL_INTAKE_PRE:
                if (elapsed(now) >= 1500) {
                    intake.spinIdle();
                    follower.followPath(paths.ShootFinal);
                    transition(State.SHOOT_FINAL_PATH, now);
                }
                break;

            case SHOOT_FINAL_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(true);
                    transition(State.SHOOT_FINAL_TRACK, now);
                }
                break;

            case SHOOT_FINAL_TRACK: {
                boolean locked = llTrack.update(true, dt);
                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT_FINAL_DUMP, now);
                }
                break;
            }

            case SHOOT_FINAL_DUMP:
                llTrack.update(true, dt);
                if (elapsed(now) >= 2000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.park);
                    transition(State.PARK, now);
                }
                break;

            // ── PARK / DONE ────────────────────────────────────────
            case PARK:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    transition(State.DONE, now);
                }
                break;

            case DONE:
                llTrack.update(false, dt);
                intake.spinIdle();
                break;
        }

        panelsTelemetry.debug("State",          state.name());
        panelsTelemetry.debug("Shooter Target", shooter.getTarget());
        panelsTelemetry.debug("Shooter Vel",    shooter.getVelocity());
        panelsTelemetry.debug("Shooter AtSpd",  shooter.atTarget());
        panelsTelemetry.debug("X",  follower.getPose().getX());
        panelsTelemetry.debug("Y",  follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    // ── Helpers ────────────────────────────────────────────────────
    private void transition(State next, long now) {
        state              = next;
        stateStartMs       = now;
        shooterStableStart = 0;
    }

    private long elapsed(long now) {
        return now - stateStartMs;
    }

    private boolean shooterReady(long now) {
        if (!shooter.isActivated() || !shooter.atTarget()) {
            shooterStableStart = 0;
            return false;
        }
        if (shooterStableStart == 0) shooterStableStart = now;
        return (now - shooterStableStart) > 200;
    }

    // ── Paths (mirrored across X axis: 144 - x) ───────────────────
    public static class Paths {

        public PathChain Shoot0;
        public PathChain Intake_A;
        public PathChain Shoot1;
        public PathChain Intake_B;
        public PathChain Shoot2;
        public PathChain Intake_C;
        public PathChain Shoot3;
        public PathChain Intake_D;
        public PathChain ShootFinal;
        public PathChain park;

        public Paths(Follower follower) {

            // Original: (56,9) → (58.125,22.362)  heading 90→110
            // Mirrored: (88,9) → (85.875,22.362)  heading 90→70
            Shoot0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.000, 9.000),
                            new Pose(85.875, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                    .build();

            // Original: (58.125,22.362)→(50.834,35.006)→(25.333,34.496)
            // Mirrored: (85.875,22.362)→(93.166,35.006)→(118.667,34.496)
            Intake_A = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(85.875, 22.362),
                            new Pose(93.166, 35.006),
                            new Pose(118.667, 34.496)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // Original: (25.333,34.496)→(58.125,22.362)  heading 180→110
            // Mirrored: (118.667,34.496)→(85.875,22.362) heading 0→70
            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(118.667, 34.496),
                            new Pose(85.875, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            // Original: (58.125,22.362)→(50.823,16.834)→(13.108,16.295)  heading 110→180
            // Mirrored: (85.875,22.362)→(93.177,16.834)→(130.892,16.295) heading 70→0
            Intake_B = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(85.875, 22.362),
                            new Pose(93.177, 16.834),
                            new Pose(130.892, 16.295)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            // Original: (12.203,13.321)→(58.125,22.362)  heading 180→110
            // Mirrored: (131.797,13.321)→(85.875,22.362) heading 0→70
            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(131.797, 13.321),
                            new Pose(85.875, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            // Original: (58.125,22.362)→(9.122,20.155)  heading 110→180
            // Mirrored: (85.875,22.362)→(134.878,20.155) heading 70→0
            Intake_C = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(85.875, 22.362),
                            new Pose(134.878, 20.155)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            // Original: (9.122,20.155)→(58.125,22.362)  heading 180→110
            // Mirrored: (134.878,20.155)→(85.875,22.362) heading 0→70
            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.878, 20.155),
                            new Pose(85.875, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            // Original: (58.125,22.362)→(10.716,9.351)  heading 110→180
            // Mirrored: (85.875,22.362)→(133.284,9.351) heading 70→0
            Intake_D = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(85.875, 22.362),
                            new Pose(133.284, 9.351)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            // Original: (10.716,9.351)→(50.126,9.742)→(59.365,18.819)  heading 180→110
            // Mirrored: (133.284,9.351)→(93.874,9.742)→(84.635,18.819) heading 0→70
            ShootFinal = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.284, 9.351),
                            new Pose(93.874, 9.742),
                            new Pose(84.635, 18.819)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            // Original: (59.365,18.819)→(35.425,21.255)  heading 110→90
            // Mirrored: (84.635,18.819)→(108.575,21.255) heading 70→90
            park = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(84.635, 18.819),
                            new Pose(108.575, 21.255)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(90))
                    .build();
        }
    }
}


