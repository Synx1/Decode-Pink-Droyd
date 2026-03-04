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
import com.seattlesolvers.solverslib.photon.PhotonCore;
import org.firstinspires.ftc.teamcode.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.SubSystem.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedCloseODo", group = "Autonomous")
@Configurable
public class RedCloseODo extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private Shooter shooter;
    private Intake intake;
    private Turret turret;

    // Target pose for shooting (basket position for red alliance)
    private static final Pose BASKET_POSE = new Pose(140, 143);

    private enum State {
        SHOOT0_PATH,
        SHOOT0_TRACK,
        SHOOT0_DUMP,

        IntakeA_PATH,
        STOP_INTAKING_PATH,
        STOP_INTAKING_WAIT,

        SHOOT1_INTAKE_PRE,
        SHOOT1_PATH,
        SHOOT1_TRACK,
        SHOOT1_DUMP,

        IntakeB_PATH,
        IntakeB_STOP,

        SHOOT2_INTAKE_PRE,
        SHOOT2_PATH,
        SHOOT2_TRACK,
        SHOOT2_DUMP,

        IntakeC_PATH,
        IntakeC_STOP,

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
    private long turretStableStart  = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(110.863, 132.694, Math.toRadians(90)));

        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
        turret  = new Turret(hardwareMap);

        paths = new Paths(follower);

        shooter.shootNear();
        shooter.clawClose();
        intake.spinIdle();
        turret.on();
        turret.resetTurret();

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
        turret.periodic();
        follower.update();

        switch (state) {

            // ── SHOOT0 ─────────────────────────────────────────────
            case SHOOT0_PATH:
                turret.face(BASKET_POSE, follower.getPose());
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    transition(State.SHOOT0_TRACK, now);
                }
                break;

            case SHOOT0_TRACK:
                turret.face(BASKET_POSE, follower.getPose());
                if (turretReady(now) && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT0_DUMP, now);
                }
                break;

            case SHOOT0_DUMP:
                turret.face(BASKET_POSE, follower.getPose());
                if (elapsed(now) >= 2000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    turret.resetTurret();  // Return turret to home position
                    follower.followPath(paths.IntakeA);
                    intake.spinIn();
                    transition(State.IntakeA_PATH, now);
                }
                break;

            // ── IntakeA ───────────────────────────────────────────
            case IntakeA_PATH:
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    follower.followPath(paths.StopIntaking);
                    transition(State.STOP_INTAKING_PATH, now);
                }
                break;

            case STOP_INTAKING_PATH:
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
                turret.face(BASKET_POSE, follower.getPose());
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    transition(State.SHOOT1_TRACK, now);
                }
                break;

            case SHOOT1_TRACK:
                turret.face(BASKET_POSE, follower.getPose());
                if (turretReady(now) && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT1_DUMP, now);
                }
                break;

            case SHOOT1_DUMP:
                turret.face(BASKET_POSE, follower.getPose());
                if (elapsed(now) >= 2000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    turret.resetTurret();
                    follower.followPath(paths.IntakeB);
                    intake.spinIn();
                    transition(State.IntakeB_PATH, now);
                }
                break;

            // ── IntakeB ───────────────────────────────────────────
            case IntakeB_PATH:
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.IntakeB_STOP, now);
                }
                break;

            case IntakeB_STOP:
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
                turret.face(BASKET_POSE, follower.getPose());
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    transition(State.SHOOT2_TRACK, now);
                }
                break;

            case SHOOT2_TRACK:
                turret.face(BASKET_POSE, follower.getPose());
                if (turretReady(now) && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT2_DUMP, now);
                }
                break;

            case SHOOT2_DUMP:
                turret.face(BASKET_POSE, follower.getPose());
                if (elapsed(now) >= 2000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    turret.resetTurret();
                    follower.followPath(paths.IntakeC);
                    intake.spinIn();
                    transition(State.IntakeC_PATH, now);
                }
                break;

            // ── IntakeC ───────────────────────────────────────────
            case IntakeC_PATH:
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.IntakeC_STOP, now);
                }
                break;

            case IntakeC_STOP:
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
                turret.face(BASKET_POSE, follower.getPose());
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    transition(State.SHOOT3_TRACK, now);
                }
                break;

            case SHOOT3_TRACK:
                turret.face(BASKET_POSE, follower.getPose());
                if (turretReady(now) && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT3_DUMP, now);
                }
                break;

            case SHOOT3_DUMP:
                turret.face(BASKET_POSE, follower.getPose());
                if (elapsed(now) >= 2000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    turret.resetTurret();
                    follower.followPath(paths.Park);
                    transition(State.PARK, now);
                }
                break;

            // ── PARK / DONE ────────────────────────────────────────
            case PARK:
                if (!follower.isBusy()) {
                    transition(State.DONE, now);
                }
                break;

            case DONE:
                intake.spinIdle();
                turret.off();
                break;
        }

        panelsTelemetry.debug("State",          state.name());
        panelsTelemetry.debug("Shooter Target", shooter.getTarget());
        panelsTelemetry.debug("Shooter Vel",    shooter.getVelocity());
        panelsTelemetry.debug("Shooter AtSpd",  shooter.atTarget());
        panelsTelemetry.debug("Turret",         turret.getTelemetryString());
        panelsTelemetry.debug("X",  follower.getPose().getX());
        panelsTelemetry.debug("Y",  follower.getPose().getY());
        panelsTelemetry.debug("H",  Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    // ── Helpers ────────────────────────────────────────────────────
    private void transition(State next, long now) {
        state              = next;
        stateStartMs       = now;
        shooterStableStart = 0;
        turretStableStart  = 0;
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

    private boolean turretReady(long now) {
        if (!turret.isReady()) {
            turretStableStart = 0;
            return false;
        }
        if (turretStableStart == 0) turretStableStart = now;
        return (now - turretStableStart) > 200;
    }

    // ── Paths ──────────────────────────────────────────────────────
    public static class Paths {
        public PathChain Shoot0;
        public PathChain IntakeA;
        public PathChain StopIntaking;
        public PathChain Shoot1;
        public PathChain IntakeB;
        public PathChain Shoot2;
        public PathChain IntakeC;
        public PathChain Shoot3;
        public PathChain Park;

        public Paths(Follower follower) {
            Shoot0 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(110.863, 132.694),
                            new Pose(83.992, 89.898)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50)).build();

            IntakeA = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(83.992, 89.898),
                            new Pose(107.675, 81.665),
                            new Pose(125.838, 81.043)
                    )
            ).setTangentHeadingInterpolation().build();

            StopIntaking = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(125.838, 81.043),
                            new Pose(111.308, 78.611),
                            new Pose(124.250, 71.511)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Shoot1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(124.250, 71.511),
                            new Pose(83.992, 89.800)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50)).build();

            IntakeB = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(83.992, 89.800),
                            new Pose(80.660, 59.867),
                            new Pose(128.237, 54.086)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0)).build();

            Shoot2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(128.237, 54.086),
                            new Pose(83.992, 89.898)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50)).build();

            IntakeC = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(83.992, 89.898),
                            new Pose(87.634, 35.851),
                            new Pose(125.251, 32.393)
                    )
            ).setTangentHeadingInterpolation().build();

            Shoot3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(125.251, 32.393),
                            new Pose(83.992, 89.898)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50)).build();

            Park = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(83.992, 89.898),
                            new Pose(113.180, 71.900)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(90)).build();
        }
    }
}