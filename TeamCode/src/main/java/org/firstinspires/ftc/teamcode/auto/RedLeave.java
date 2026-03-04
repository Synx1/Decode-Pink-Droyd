package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Simple Path Test", group = "Autonomous")
public class RedLeave extends OpMode {
    public Follower follower;
    private PathChain path1;
    private boolean pathStarted = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(86.805, 8.652, Math.toRadians(90)));

        // Build Path1
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.805, 8.652),
                                new Pose(106.172, 9.285)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        telemetry.addLine("Initialized - Ready to run Path1");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(path1);
        pathStarted = true;
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("Path Started", pathStarted);
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        if (!follower.isBusy() && pathStarted) {
            telemetry.addLine("Path Complete!");
        }

        telemetry.update();
    }
}