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

@Autonomous(name = "RedClose", group = "Autonomous")
@Configurable
public class RedClose extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private Shooter shooter;
    private Intake intake;
    private LLtrack llTrack;

    // delay after each intake path
    private static final long INTAKE_PATH_DELAY_MS = 250;

    private enum State {
        SHOOT0_PATH,
        SHOOT0_TRACK,
        SHOOT0_DUMP,

        IntakeA_PATH,
        IntakeA_WAIT,

        SHOOT1_INTAKE_PRE,
        SHOOT1_PATH,
        SHOOT1_TRACK,
        SHOOT1_DUMP,

        IntakeB_PATH,
        IntakeB_WAIT,

        SHOOT2_INTAKE_PRE,
        SHOOT2_PATH,
        SHOOT2_TRACK,
        SHOOT2_DUMP,

        IntakeC_PATH,
        IntakeC_WAIT,

        SHOOT3_INTAKE_PRE,
        SHOOT3_PATH,
        SHOOT3_TRACK,
        SHOOT3_DUMP,

        PARK,
        DONE
    }

    private State state = State.SHOOT0_PATH;
    private long stateStartMs = 0;
    private long lastLoopMs = 0;
    private long shooterStableStart = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(110.863, 132.694, Math.toRadians(90)));

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        llTrack = new LLtrack(hardwareMap, false);

        paths = new Paths(follower);

        shooter.shootNear();
        shooter.clawClose();
        intake.spinIdle();
        llTrack.setAlliance(false);

        lastLoopMs = System.currentTimeMillis();
        stateStartMs = lastLoopMs;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.followPath(paths.Shoot0);
        state = State.SHOOT0_PATH;
        stateStartMs = System.currentTimeMillis();
        lastLoopMs = stateStartMs;
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        double dt = (now - lastLoopMs) / 1000.0;
        lastLoopMs = now;

        shooter.periodic();
        follower.update();

        switch (state) {

            // ----------------- SHOOT 0 -----------------

            case SHOOT0_PATH:
                // turret: GO HOME while driving
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(false);
                    transition(State.SHOOT0_TRACK, now);
                }
                break;

            case SHOOT0_TRACK: {
                // turret: TRACK / AIM
                boolean locked = llTrack.update(true, dt);
                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT0_DUMP, now);
                }
                break;
            }

            case SHOOT0_DUMP:
                // turret: still TRACK while dumping
                llTrack.update(true, dt);
                if (elapsed(now) >= 1000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.IntakeA);
                    intake.spinIn();
                    transition(State.IntakeA_PATH, now);
                }
                break;

            // ----------------- INTAKE A -----------------

            case IntakeA_PATH:
                // turret: GO HOME during intake path A
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.IntakeA_WAIT, now);
                }
                break;

            case IntakeA_WAIT:
                // turret: GO HOME during small wait
                llTrack.update(false, dt);
                if (elapsed(now) >= INTAKE_PATH_DELAY_MS) {
                    intake.spinIn();
                    transition(State.SHOOT1_INTAKE_PRE, now);
                }
                break;

            // ----------------- SHOOT 1 -----------------

            case SHOOT1_INTAKE_PRE:
                // turret: GO HOME while pre-intaking for shoot 1
                llTrack.update(false, dt);
                if (elapsed(now) >= 3000) {
                    intake.spinIdle();
                    follower.followPath(paths.Shoot1);
                    transition(State.SHOOT1_PATH, now);
                }
                break;

            case SHOOT1_PATH:
                // turret: GO HOME while driving to shoot 1 pose
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(false);
                    transition(State.SHOOT1_TRACK, now);
                }
                break;

            case SHOOT1_TRACK: {
                // turret: TRACK / AIM at tag
                boolean locked = llTrack.update(true, dt);
                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT1_DUMP, now);
                }
                break;
            }

            case SHOOT1_DUMP:
                // turret: TRACK while dumping
                llTrack.update(true, dt);
                if (elapsed(now) >= 1000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.IntakeB);
                    intake.spinIn();
                    transition(State.IntakeB_PATH, now);
                }
                break;

            // ----------------- INTAKE B -----------------

            case IntakeB_PATH:
                // turret: GO HOME during intake B path
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.IntakeB_WAIT, now);
                }
                break;

            case IntakeB_WAIT:
                // turret: GO HOME during wait
                llTrack.update(false, dt);
                if (elapsed(now) >= INTAKE_PATH_DELAY_MS) {
                    intake.spinIn();
                    transition(State.SHOOT2_INTAKE_PRE, now);
                }
                break;

            // ----------------- SHOOT 2 -----------------

            case SHOOT2_INTAKE_PRE:
                // turret: GO HOME
                llTrack.update(false, dt);
                if (elapsed(now) >= 3000) {
                    intake.spinIdle();
                    follower.followPath(paths.Shoot2);
                    transition(State.SHOOT2_PATH, now);
                }
                break;

            case SHOOT2_PATH:
                // turret: GO HOME while driving
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(false);
                    transition(State.SHOOT2_TRACK, now);
                }
                break;

            case SHOOT2_TRACK: {
                // turret: TRACK / AIM
                boolean locked = llTrack.update(true, dt);
                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT2_DUMP, now);
                }
                break;
            }

            case SHOOT2_DUMP:
                // turret: TRACK while dumping
                llTrack.update(true, dt);
                if (elapsed(now) >= 1000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.IntakeC);
                    intake.spinIn();
                    transition(State.IntakeC_PATH, now);
                }
                break;

            // ----------------- INTAKE C -----------------

            case IntakeC_PATH:
                // turret: GO HOME during intake C path
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.IntakeC_WAIT, now);
                }
                break;

            case IntakeC_WAIT:
                // turret: GO HOME during wait
                llTrack.update(false, dt);
                if (elapsed(now) >= INTAKE_PATH_DELAY_MS) {
                    intake.spinIn();
                    transition(State.SHOOT3_INTAKE_PRE, now);
                }
                break;

            // ----------------- SHOOT 3 -----------------

            case SHOOT3_INTAKE_PRE:
                // turret: GO HOME
                llTrack.update(false, dt);
                if (elapsed(now) >= 3000) {
                    intake.spinIdle();
                    follower.followPath(paths.Shoot3);
                    transition(State.SHOOT3_PATH, now);
                }
                break;

            case SHOOT3_PATH:
                // turret: GO HOME while driving
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    llTrack.setAlliance(false);
                    transition(State.SHOOT3_TRACK, now);
                }
                break;

            case SHOOT3_TRACK: {
                // turret: TRACK / AIM
                boolean locked = llTrack.update(true, dt);
                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    transition(State.SHOOT3_DUMP, now);
                }
                break;
            }

            case SHOOT3_DUMP:
                // turret: TRACK while dumping
                llTrack.update(true, dt);
                if (elapsed(now) >= 1000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.Park);
                    transition(State.PARK, now);
                }
                break;

            // ----------------- PARK & DONE -----------------

            case PARK:
                // turret: GO HOME while parking
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    transition(State.DONE, now);
                }
                break;

            case DONE:
                // turret: GO HOME at the end
                llTrack.update(false, dt);
                intake.spinIdle();
                break;
        }

        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("Shooter Target", shooter.getTarget());
        panelsTelemetry.debug("Shooter Vel", shooter.getVelocity());
        panelsTelemetry.debug("Shooter AtSpd", shooter.atTarget());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    private void transition(State next, long now) {
        state = next;
        stateStartMs = now;
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
        return (now - shooterStableStart) > 100;
    }

    public static class Paths {
        public PathChain Shoot0;
        public PathChain IntakeA;
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
                                    new Pose(85.729, 84.034)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))
                    .build();

            IntakeA = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.729, 84.034),
                                    new Pose(125.404, 82.129)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.404, 82.129),
                                    new Pose(85.947, 84.370)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            IntakeB = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(85.947, 84.370),
                                    new Pose(81.746, 50.528),
                                    new Pose(122.156, 54.737)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.156, 54.737),
                                    new Pose(86.164, 84.034)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            IntakeC = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.164, 84.034),
                                    new Pose(87.634, 35.851),
                                    new Pose(119.387, 32.176)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.387, 32.176),
                                    new Pose(85.729, 84.034)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.729, 84.034),
                                    new Pose(113.180, 71.900)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(90))
                    .build();
        }
    }
}