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

@Autonomous(name = "RedFar", group = "Autonomous")
@Configurable
public class RedFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private Shooter shooter;
    private Intake intake;
    private LLtrack llTrack;

    // ── State Machine ──────────────────────────────────────────────
    private enum State {
        SHOOT0_PATH,
        SHOOT0_TRACK,
        SHOOT0_DUMP,

        INTAKE_A_PATH,
        INTAKE_A_STOP,

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

    // ── Init ───────────────────────────────────────────────────────
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(86.760, 9.063, Math.toRadians(90)));

        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
        llTrack = new LLtrack(hardwareMap, false); // false = red alliance

        paths = new Paths(follower);

        shooter.shootFar();
        shooter.clawClose();
        intake.spinIdle();
        llTrack.setAlliance(false);

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

            case SHOOT0_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(false);
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
                    llTrack.setAlliance(false);
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
                    llTrack.setAlliance(false);
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
                    llTrack.setAlliance(false);
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
        public PathChain Shoot1;
        public PathChain Intake_B;
        public PathChain Shoot2;
        public PathChain Intake_C;
        public PathChain Shoot3;
        public PathChain Park;

        public Paths(Follower follower) {

            Shoot0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(86.760, 9.063),
                            new Pose(85.875, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                    .build();

            Intake_A = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(85.875, 22.362),
                            new Pose(99.701, 37.375),
                            new Pose(124.646, 35.978)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.646, 35.978),
                            new Pose(85.875, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            Intake_B = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(85.875, 22.362),
                            new Pose(89.317, 60.480),
                            new Pose(125.756, 59.661)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125.756, 59.661),
                            new Pose(85.875, 22.362)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            Intake_C = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(85.875, 22.362),
                            new Pose(84.841, 86.435),
                            new Pose(123.642, 84.166)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(123.642, 84.166),
                            new Pose(86.166, 22.343)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            Park = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(86.166, 22.343),
                            new Pose(110.465, 22.708)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(90))
                    .build();
        }
    }
}

