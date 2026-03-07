package org.firstinspires.ftc.teamcode.scraps;

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

@Autonomous(name = "BlueClose", group = "Autonomous")
@Configurable
public class BlueClose extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private Shooter shooter;
    private Intake intake;
    private LLtrack llTrack;

    private enum State {
        SHOOT0_PATH,
        SHOOT0_TRACK,
        SHOOT0_DUMP,

        INTAKE_A_PATH,
        STOP_INTAKING_PATH,
        STOP_INTAKING_WAIT,

        SHOOT1_INTAKE_PRE,
        SHOOT1_PATH,
        SHOOT1_TRACK,
        SHOOT1_DUMP,

        INTAKE_B_PATH,
        INTAKE_B_STOP,

        SHOOT2_INTAKE_PRE,
        SHOOT2_PATH,
        SHOOT2_TRACK,
        SHOOT2_DUMP,

        INTAKE_C_PATH,
        INTAKE_C_STOP,

        SHOOT3_INTAKE_PRE,
        SHOOT3_PATH,
        SHOOT3_TRACK,
        SHOOT3_DUMP,

        PARK,
        DONE
    }

    private State state = State.SHOOT0_PATH;
    private long stateStartMs       = 0;
    private long lastLoopMs         = 0;
    private long shooterStableStart = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.137, 132.694, Math.toRadians(90)));

        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
        llTrack = new LLtrack(hardwareMap, true); // blue alliance

        paths = new Paths(follower);

        shooter.shootNear();
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
                    intake.spinIn();               // intake ON while driving Intake_A
                    transition(State.INTAKE_A_PATH, now);
                }
                break;

            // ── INTAKE_A ───────────────────────────────────────────
            case INTAKE_A_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();             // intake OFF at START of StopIntaking
                    follower.followPath(paths.StopIntaking);
                    transition(State.STOP_INTAKING_PATH, now);
                }
                break;

            case STOP_INTAKING_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    transition(State.STOP_INTAKING_WAIT, now);
                }
                break;

            case STOP_INTAKING_WAIT:
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
                    follower.followPath(paths.Park);
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

    // ── Paths ──────────────────────────────────────────────────────
    public static class Paths {

        public PathChain Shoot0;
        public PathChain Intake_A;
        public PathChain StopIntaking;
        public PathChain Shoot1;
        public PathChain Intake_B;
        public PathChain Shoot2;
        public PathChain Intake_C;
        public PathChain Shoot3;
        public PathChain Park;

        public Paths(Follower follower) {

            Shoot0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(33.137, 132.694),
                            new Pose(62.100, 83.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))
                    .build();

            Intake_A = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(62.100, 83.000),
                            new Pose(20.100, 82.400)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            StopIntaking = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(20.100, 82.400),
                            new Pose(32.692, 81.906),
                            new Pose(18.200, 75.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.200, 75.000),
                            new Pose(62.100, 83.100)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            Intake_B = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(62.100, 83.100),
                            new Pose(60.045, 59.867),
                            new Pose(22.934, 59.900)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22.934, 59.900),
                            new Pose(62.100, 82.400)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            Intake_C = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(62.100, 82.400),
                            new Pose(56.366, 35.851),
                            new Pose(23.400, 35.300)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(23.400, 35.300),
                            new Pose(61.900, 82.800)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            Park = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.900, 82.800),
                            new Pose(25.200, 71.900)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(90))
                    .build();
        }
    }
}

