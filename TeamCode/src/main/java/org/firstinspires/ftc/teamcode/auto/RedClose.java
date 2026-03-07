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

    private enum State {
        SHOOT0_PATH,
        SHOOT0_TRACK,
        SHOOT0_DUMP,

        IntakeA_PATH,
        OPENGATE_PATH,

        SHOOT1_PATH,
        SHOOT1_TRACK,
        SHOOT1_DUMP,

        IntakeB_PATH,
        SHOOT2_INTAKE_PRE,
        SHOOT2_PATH,
        SHOOT2_TRACK,
        SHOOT2_DUMP,

        IntakeC_PATH,
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
                if (elapsed(now) >= 1150) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.IntakeA);
                    intake.spinIn();
                    transition(State.IntakeA_PATH, now);
                }
                break;

            // ----------------- INTAKE A -----------------

            case IntakeA_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle(); // ONLY off for opengate
                    follower.followPath(paths.opengate);
                    transition(State.OPENGATE_PATH, now);
                }
                break;

            // ----------------- OPEN GATE -----------------

            case OPENGATE_PATH:
                llTrack.update(false, dt);
                intake.spinIdle(); // intake off only here
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot1);
                    transition(State.SHOOT1_PATH, now);
                }
                break;

            // ----------------- SHOOT 1 -----------------

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
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.SHOOT2_INTAKE_PRE, now);
                }
                break;

            // ----------------- SHOOT 2 -----------------

            case SHOOT2_INTAKE_PRE:
                llTrack.update(false, dt);
                if (elapsed(now) >= 150) {
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
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    transition(State.SHOOT3_INTAKE_PRE, now);
                }
                break;

            // ----------------- SHOOT 3 -----------------

            case SHOOT3_INTAKE_PRE:
                llTrack.update(false, dt);
                if (elapsed(now) >= 150) {
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
                if (elapsed(now) >= 1000) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.Park);
                    transition(State.PARK, now);
                }
                break;

            // ----------------- PARK & DONE -----------------

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
        public PathChain opengate;
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

                                    new Pose(88.683, 85.326)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))

                    .build();

            IntakeA = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.683, 85.326),
                                    new Pose(102.582, 81.669),
                                    new Pose(126.807, 81.237)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            opengate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(126.807, 81.237),
                                    new Pose(104.331, 75.704),
                                    new Pose(125.994, 72.480)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.994, 72.480),

                                    new Pose(88.764, 85.276)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                    .build();

            IntakeB = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.764, 85.276),
                                    new Pose(83.955, 59.867),
                                    new Pose(126.105, 52.923)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.105, 52.923),

                                    new Pose(88.490, 85.501)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                    .build();

            IntakeC = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.490, 85.501),
                                    new Pose(87.634, 35.851),
                                    new Pose(122.344, 31.424)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.344, 31.424),

                                    new Pose(88.883, 85.513)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.883, 85.513),

                                    new Pose(120.932, 70.931)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(90))

                    .build();
        }
    }

}

