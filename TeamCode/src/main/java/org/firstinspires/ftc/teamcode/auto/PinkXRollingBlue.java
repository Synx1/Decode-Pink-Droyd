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

@Autonomous(name = "PinkXRollingBlue", group = "Autonomous")
@Configurable
public class PinkXRollingBlue extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private Shooter shooter;
    private Intake intake;
    private LLtrack llTrack;

    // ── State Machine ──────────────────────────────────────────────
    private enum State {
        // Shoot0 sequence
        SHOOT0_PATH,        // drive to first shoot position
        SHOOT0_TRACK,       // LL lock
        SHOOT0_DUMP,        // intake 2s then stop, close claw

        // Intake_A sequence
        INTAKE_A_PATH,      // drive to intake spot, intake on
        INTAKE_A_STOP,      // path done → stop intake

        // Shoot1 sequence
        SHOOT1_INTAKE_PRE,  // intake 1.5s before driving back
        SHOOT1_PATH,        // drive back to shoot
        SHOOT1_TRACK,       // LL lock
        SHOOT1_DUMP,        // intake 2s then stop, close claw

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

        // Final Shoot (reuse Shoot3 logic after Intake_D)
        SHOOT_FINAL_INTAKE_PRE,
        SHOOT_FINAL_PATH,
        SHOOT_FINAL_TRACK,
        SHOOT_FINAL_DUMP,

        PARK,
        DONE
    }

    private State state = State.SHOOT0_PATH;
    private long stateStartMs = 0;
    private long lastLoopMs   = 0;
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

        shooter.shootFar();   // spin up early
        shooter.clawClose();
        intake.spinIdle();
        llTrack.setAlliance(true);

        lastLoopMs   = System.currentTimeMillis();
        stateStartMs = lastLoopMs;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update();
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
                    intake.spinIn();           // start intake during Intake_A path
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
                // brief settle, then start Shoot1 pre-intake
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
        state        = next;
        stateStartMs = now;
        shooterStableStart = 0; // reset stable timer on every transition
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

    // ── Paths ──────────────────────────────────────────────────────
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

            Shoot0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 9.000),
                            new Pose(58.125, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                    .build();

            Intake_A = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58.125, 22.362),
                            new Pose(50.834, 35.006),
                            new Pose(25.333, 34.496)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(25.333, 34.496),
                            new Pose(58.125, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Intake_B = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58.125, 22.362),
                            new Pose(50.823, 16.834),
                            new Pose(13.108, 16.295)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.203, 13.321),
                            new Pose(58.125, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Intake_C = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(58.125, 22.362),
                            new Pose(9.122, 20.155)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.122, 20.155),
                            new Pose(58.125, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Intake_D = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(58.125, 22.362),
                            new Pose(10.716, 9.351)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            ShootFinal = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(10.716, 9.351),
                            new Pose(50.126, 9.742),
                            new Pose(59.365, 18.819)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(59.365, 18.819),
                            new Pose(35.425, 21.255)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(90))
                    .build();
        }
    }
}