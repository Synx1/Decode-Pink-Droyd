package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystem.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.SubSystem.LLtrack;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FrankLover29000", group = "Main")
public class FrankLoverBlue extends OpMode {

    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private LLtrack llTrack;
    private Follower follower;
    private FarPaths paths;

    private enum State {
        RUN_PATH1,
        TRACK1,     // end of P1: open claw + LL
        DUMP1,      // intake 3s

        RUN_PATH2,  // drive P2 while intaking
        RUN_PATH3,  // drive P3
        TRACK2,     // end of P3: open claw + LL
        DUMP2,      // intake 3s, then intake along P4–7

        RUN_PATH4,  // intake ON
        RUN_PATH5,  // intake ON
        RUN_PATH6,  // intake ON
        RUN_PATH7,  // intake ON
        TRACK3,     // end of P7: open claw + LL
        DUMP3,      // intake 3s, then intake along P8–9

        RUN_PATH8,  // intake ON
        RUN_PATH9,  // intake ON
        TRACK4,     // end of P9: open claw + LL
        DUMP4,      // intake 3s, then go P10

        RUN_PATH10, // final move
        DONE
    }

    private State state = State.RUN_PATH1;

    private long lastLoopMs = 0;
    private long stateStartMs = 0;
    private long shooterStableStart = 0;

    // For 0.5 second delay at end of every path
    private long pathDelayStartMs = 0;
    private static final long PATH_DELAY_MS = 200;
    private static final long PATH6_TIMEOUT_MS = 4000;   // adjust as needed (2.5s)

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        Pose startPose = new Pose(64.000, 8.000, Math.toRadians(90));
        follower.setStartingPose(startPose);

        driveTrain = new DriveTrain(hardwareMap, startPose);
        shooter    = new Shooter(hardwareMap);
        intake     = new Intake(hardwareMap);
        llTrack    = new LLtrack(hardwareMap, true);

        paths = new FarPaths(follower);

        // Spin up shooter to FAR preset immediately
        shooter.shootFar();

        lastLoopMs   = System.currentTimeMillis();
        stateStartMs = lastLoopMs;
        pathDelayStartMs = 0;

        telemetry.addLine("INIT OK - Shooter FAR preset active.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(paths.FirstAction()); // Path1
        state = State.RUN_PATH1;
        stateStartMs = System.currentTimeMillis();
        lastLoopMs   = stateStartMs;
        pathDelayStartMs = 0;

        // safe defaults
        shooter.clawClose();
        intake.spinIdle();
        llTrack.setAlliance(true);
    }

    @Override
    public void loop() {

        long now = System.currentTimeMillis();
        double dt = (now - lastLoopMs) / 1000.0;
        lastLoopMs = now;

        shooter.periodic();
        follower.update();

        switch (state) {

            // ------------------------------------------------------------
            // PATH 1 : drive near scoring spot
            // ------------------------------------------------------------
            case RUN_PATH1:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        // End of P1: open claw + start LL alignment
                        llTrack.setAlliance(true);
                        shooter.clawOpen();

                        pathDelayStartMs = 0;
                        state = State.TRACK1;
                        stateStartMs = now;
                    }
                }
                break;

            // End of P1: LL alignment, then intake
            case TRACK1: {
                boolean locked = llTrack.update(true, dt);

                telemetry.addData("TRACK1 tx", llTrack.getAllianceTx());
                telemetry.addData("TRACK1 locked", locked);

                if (locked && shooterReady(now)) {
                    // Start intake once aligned
                    intake.spinIn();
                    state = State.DUMP1;
                    stateStartMs = now;
                }
                break;
            }

            // Intake for 3 seconds after P1 LL
            case DUMP1:
                llTrack.update(true, dt);

                if (now - stateStartMs >= 3000) {
                    shooter.clawClose();   // close after dump

                    // Drive Path2 while intaking
                    intake.spinIn();
                    follower.followPath(paths.Path2());
                    pathDelayStartMs = 0;

                    state = State.RUN_PATH2;
                    stateStartMs = now;
                }
                break;

            // ------------------------------------------------------------
            // PATH 2 : drive while intaking
            // ------------------------------------------------------------
            case RUN_PATH2:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        // after P2 -> stop intake, then do P3
                        intake.spinIdle();
                        follower.followPath(paths.Path3());
                        pathDelayStartMs = 0;

                        state = State.RUN_PATH3;
                        stateStartMs = now;
                    }
                }
                break;

            // ------------------------------------------------------------
            // PATH 3 : drive back, then LL again
            // ------------------------------------------------------------
            case RUN_PATH3:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        // End of P3: open claw + LL align
                        shooter.clawOpen();
                        llTrack.setAlliance(true);

                        pathDelayStartMs = 0;
                        state = State.TRACK2;
                        stateStartMs = now;
                    }
                }
                break;

            // End of P3: LL alignment, then intake
            case TRACK2: {
                boolean locked = llTrack.update(true, dt);

                telemetry.addData("TRACK2 tx", llTrack.getAllianceTx());
                telemetry.addData("TRACK2 locked", locked);

                if (locked && shooterReady(now)) {
                    // Start intake after alignment
                    intake.spinIn();
                    state = State.DUMP2;
                    stateStartMs = now;
                }
                break;
            }

            // Intake for 3 seconds after P3 LL, then go P4–7 with intake
            case DUMP2:
                llTrack.update(true, dt);

                if (now - stateStartMs >= 3000) {
                    // after 3s: close claw, keep intake ON for P4–7
                    shooter.clawClose();
                    intake.spinIn();

                    follower.followPath(paths.Path4());
                    pathDelayStartMs = 0;

                    state = State.RUN_PATH4;
                    stateStartMs = now;
                }
                break;

            // ============================================================
            // PATHS 4–7 : intake ON the whole time, LL + dump at end of P7
            // ============================================================

            case RUN_PATH4:
                llTrack.update(false, dt);
                intake.spinIn();
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        follower.followPath(paths.Path5());
                        pathDelayStartMs = 0;

                        state = State.RUN_PATH5;
                        stateStartMs = now;
                    }
                }
                break;

            case RUN_PATH5:
                llTrack.update(false, dt);
                intake.spinIn();
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        follower.followPath(paths.Path6());
                        pathDelayStartMs = 0;

                        state = State.RUN_PATH6;
                        stateStartMs = now;
                    }
                }
                break;

            case RUN_PATH6:
                llTrack.update(false, dt);
                intake.spinIn();

                // ---- TIMEOUT CHECK ----
                if (now - stateStartMs > PATH6_TIMEOUT_MS) {
                    // Force exit of Path6
                    follower.breakFollowing();
                    shooter.clawOpen();
                    llTrack.setAlliance(true);

                    pathDelayStartMs = 0;
                    state = State.TRACK3;  // go directly to next phase
                    stateStartMs = now;
                    break;
                }

                // ---- NORMAL EXIT CHECK ----
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        follower.followPath(paths.Path7());
                        pathDelayStartMs = 0;

                        state = State.RUN_PATH7;
                        stateStartMs = now;
                    }
                }
                break;

            // PATH 7 with intake ON; at end, open claw + LL align
            case RUN_PATH7:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        // End of P7: open claw + LL
                        shooter.clawOpen();
                        llTrack.setAlliance(true);

                        pathDelayStartMs = 0;
                        state = State.TRACK3;
                        stateStartMs = now;
                    }
                }
                break;

            // End of P7: LL alignment, then intake
            case TRACK3: {
                boolean locked = llTrack.update(true, dt);

                telemetry.addData("TRACK3 tx", llTrack.getAllianceTx());
                telemetry.addData("TRACK3 locked", locked);

                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    state = State.DUMP3;
                    stateStartMs = now;
                }
                break;
            }

            // Intake 3s after P7 LL, then go P8–9 with intake ON
            case DUMP3:
                llTrack.update(true, dt);

                if (now - stateStartMs >= 3000) {
                    shooter.clawClose();
                    // keep intake ON for P8 and P9
                    intake.spinIn();

                    follower.followPath(paths.Path8());
                    pathDelayStartMs = 0;

                    state = State.RUN_PATH8;
                    stateStartMs = now;
                }
                break;

            // ============================================================
            // PATHS 8–9 : intake ON, LL + dump at end of P9, then P10
            // ============================================================

            case RUN_PATH8:
                llTrack.update(false, dt);
                // intake still spinIn()
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        follower.followPath(paths.Path9());
                        pathDelayStartMs = 0;

                        state = State.RUN_PATH9;
                        stateStartMs = now;
                    }
                }
                break;

            case RUN_PATH9:
                llTrack.update(false, dt);
                // intake still spinIn()
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        // End of P9: open claw + LL align
                        shooter.clawOpen();
                        llTrack.setAlliance(true);

                        pathDelayStartMs = 0;
                        state = State.TRACK4;
                        stateStartMs = now;
                    }
                }
                break;

            // End of P9: LL alignment, then intake
            case TRACK4: {
                boolean locked = llTrack.update(true, dt);

                telemetry.addData("TRACK4 tx", llTrack.getAllianceTx());
                telemetry.addData("TRACK4 locked", locked);

                if (locked && shooterReady(now)) {
                    intake.spinIn();
                    state = State.DUMP4;
                    stateStartMs = now;
                }
                break;
            }

            // Intake 3s after P9 LL, then go Path10
            case DUMP4:
                llTrack.update(true, dt);

                if (now - stateStartMs >= 3000) {
                    intake.spinIdle();
                    shooter.clawClose();

                    follower.followPath(paths.Path10());
                    pathDelayStartMs = 0;

                    state = State.RUN_PATH10;
                    stateStartMs = now;
                }
                break;

            // Final move
            case RUN_PATH10:
                llTrack.update(false, dt);
                if (!follower.isBusy()) {
                    if (pathDelayStartMs == 0) {
                        pathDelayStartMs = now;
                    } else if (now - pathDelayStartMs >= PATH_DELAY_MS) {
                        state = State.DONE;
                        pathDelayStartMs = 0;
                        stateStartMs = now;
                    }
                }
                break;

            case DONE:
                llTrack.update(false, dt);
                intake.spinIdle();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Shooter Target", shooter.getTarget());
        telemetry.addData("Shooter Vel", shooter.getVelocity());
        telemetry.addData("Shooter AtSpeed", shooter.atTarget());
        telemetry.update();
    }

    // ============================================================
    // Shooter stable speed check (200ms requirement)
    // ============================================================
    private boolean shooterReady(long now) {

        if (!shooter.isActivated() || !shooter.atTarget()) {
            shooterStableStart = 0;
            return false;
        }

        if (shooterStableStart == 0) {
            shooterStableStart = now;
        }

        return (now - shooterStableStart) > 200;
    }

    // ============================================================
    // Paths – matches your PedroAutonomous Paths 1–10
    // ============================================================
    private static class FarPaths {

        private final Follower f;

        public Pose p1Start = new Pose(64.000, 8.000, Math.toRadians(90));
        public Pose p1End   = new Pose(60.600, 16.260, Math.toRadians(110));

        public FarPaths(Follower follower) {
            this.f = follower;
        }

        // Path1
        public PathChain FirstAction() {
            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    p1Start,
                                    p1End
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(110)
                    )
                    .build();
        }

        // Path2
        public PathChain Path2() {
            return f.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.600, 17.782),
                                    new Pose(57.781, 61.570),
                                    new Pose(17.822, 59.813)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(180)
                    )
                    .build();
        }

        // Path3
        public PathChain Path3() {
            return f.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(17.822, 59.813),
                                    new Pose(57.649, 61.605),
                                    new Pose(60.651, 17.782)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(110)
                    )
                    .build();
        }

        // Path4
        public PathChain Path4() {
            return f.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.651, 17.782),
                                    new Pose(36.964, 11.843),
                                    new Pose(11.5, 12.9)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }

        // Path5
        public PathChain Path5() {
            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.5, 12.9),
                                    new Pose(16.481927710843358, 12.927710843373504)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }

        // Path6
        public PathChain Path6() {
            return f.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.482, 12.928),
                                    new Pose(20.904, 10.458),
                                    new Pose(10.193, 10.205)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180)
                    )
                    .build();
        }

        // Path7
        public PathChain Path7() {
            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.193, 10.205),
                                    new Pose(60.700, 17.782)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(110)
                    )
                    .build();
        }

        // Path8
        public PathChain Path8() {
            return f.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.700, 17.782),
                                    new Pose(62.573, 35.192),
                                    new Pose(14.687, 35.735)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }

        // Path9
        public PathChain Path9() {
            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(14.687, 35.735),
                                    new Pose(60.700, 17.782)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(110)
                    )
                    .build();
        }

        // Path10
        public PathChain Path10() {
            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(60.700, 17.782),
                                    new Pose(33.410, 18.735)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(110),
                            Math.toRadians(90)
                    )
                    .build();
        }
    }
}