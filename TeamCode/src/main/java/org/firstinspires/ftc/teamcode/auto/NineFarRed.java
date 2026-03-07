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

@Autonomous(name = "NineFarRed", group = "Autonomous")
@Configurable
public class NineFarRed extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private Shooter shooter;
    private Intake intake;
    private LLtrack llTrack;

    private static final long SHOOT_SETTLE_MS = 550;
    private static final long DUMP0_MS = 1150;
    private static final long DUMP1_MS = 1000;
    private static final long DUMP2_MS = 1000;

    private enum State {
        SHOOT0_PATH,
        SHOOT0_TRACK,
        SHOOT0_DUMP,

        INTAKEA_PATH,

        SHOOT1_PATH,
        SHOOT1_TRACK,
        SHOOT1_DUMP,

        INTAKEB_PATH,

        SHOOT2_PATH,
        SHOOT2_TRACK,
        SHOOT2_DUMP,

        PATH6,
        DONE
    }

    private State state = State.SHOOT0_PATH;
    private long stateStartMs = 0;
    private long lastLoopMs = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(81.195, 9.163, Math.toRadians(90)));

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        // Force LLtrack to only use AprilTag 20
        LLtrack.BLUE_TAG_ID = 20;
        LLtrack.RED_TAG_ID = 20;

        llTrack = new LLtrack(hardwareMap, false);
        llTrack.setAlliance(false);

        paths = new Paths(follower);

        shooter.shootFar();
        shooter.clawClose();
        intake.spinIdle();

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

            case SHOOT0_PATH:
                // turret locked / home while driving
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    transition(State.SHOOT0_TRACK, now);
                }
                break;

            case SHOOT0_TRACK:
                // unlock turret and track only after path ends
                llTrack.update(true, dt);
                if (elapsed(now) >= SHOOT_SETTLE_MS && shooter.atTarget()) {
                    intake.spinIn();
                    transition(State.SHOOT0_DUMP, now);
                }
                break;

            case SHOOT0_DUMP:
                llTrack.update(true, dt);
                if (elapsed(now) >= DUMP0_MS) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.IntakeA);
                    intake.spinIn();
                    transition(State.INTAKEA_PATH, now);
                }
                break;

            case INTAKEA_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    follower.followPath(paths.Shoot1);
                    transition(State.SHOOT1_PATH, now);
                }
                break;

            case SHOOT1_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    transition(State.SHOOT1_TRACK, now);
                }
                break;

            case SHOOT1_TRACK:
                llTrack.update(true, dt);
                if (elapsed(now) >= SHOOT_SETTLE_MS && shooter.atTarget()) {
                    intake.spinIn();
                    transition(State.SHOOT1_DUMP, now);
                }
                break;

            case SHOOT1_DUMP:
                llTrack.update(true, dt);
                if (elapsed(now) >= DUMP1_MS) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.IntakeB);
                    intake.spinIn();
                    transition(State.INTAKEB_PATH, now);
                }
                break;

            case INTAKEB_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    intake.spinIdle();
                    follower.followPath(paths.shoot2);
                    transition(State.SHOOT2_PATH, now);
                }
                break;

            case SHOOT2_PATH:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    shooter.clawOpen();
                    transition(State.SHOOT2_TRACK, now);
                }
                break;

            case SHOOT2_TRACK:
                llTrack.update(true, dt);
                if (elapsed(now) >= SHOOT_SETTLE_MS && shooter.atTarget()) {
                    intake.spinIn();
                    transition(State.SHOOT2_DUMP, now);
                }
                break;

            case SHOOT2_DUMP:
                llTrack.update(true, dt);
                if (elapsed(now) >= DUMP2_MS) {
                    intake.spinIdle();
                    shooter.clawClose();
                    follower.followPath(paths.Path6);
                    transition(State.PATH6, now);
                }
                break;

            case PATH6:
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
        panelsTelemetry.debug("Tag Blue", LLtrack.BLUE_TAG_ID);
        panelsTelemetry.debug("Tag Red", LLtrack.RED_TAG_ID);
        panelsTelemetry.update(telemetry);
    }

    private void transition(State next, long now) {
        state = next;
        stateStartMs = now;
    }

    private long elapsed(long now) {
        return now - stateStartMs;
    }

    public static class Paths {
        public PathChain Shoot0;
        public PathChain IntakeA;
        public PathChain Shoot1;
        public PathChain IntakeB;
        public PathChain shoot2;
        public PathChain Path6;

        public Paths(Follower follower) {
            Shoot0 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(81.195, 9.163),
                                    new Pose(88.000, 18.388)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                    .build();

            IntakeA = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 18.388),
                                    new Pose(94.264, 35.476),
                                    new Pose(130.824, 34.742)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.824, 34.742),
                                    new Pose(88.140, 18.388)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            IntakeB = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.140, 18.388),
                                    new Pose(92.409, 55.571),
                                    new Pose(123.494, 56.635)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.494, 56.635),
                                    new Pose(88.200, 18.388)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.200, 18.388),
                                    new Pose(111.194, 18.751)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(90))
                    .build();
        }
    }
}