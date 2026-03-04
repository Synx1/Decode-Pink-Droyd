package org.firstinspires.ftc.teamcode.SubSystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveTrain {

    private final Follower follower;
    private boolean hold = false;

    // true  = robot-centric
    // false = field-centric
    private boolean robotCentric = true;  // default: robot-centric

    public DriveTrain(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    public void startTeleOp() {
        follower.startTeleopDrive();
    }

    public CommandBuilder start() {
        return Commands.instant(this::startTeleOp);
    }

    public void periodic() {
        follower.update();
    }

    public void drive(Gamepad gamepad) {
        if (!hold) {
            // FTC gamepad: up on stick = negative Y
            double forward = -gamepad.left_stick_y;  // forward/back
            double strafe  = -gamepad.left_stick_x;  // left/right
            double turn    = -gamepad.right_stick_x; // rotate

            follower.setTeleOpDrive(
                    forward,
                    strafe,
                    turn,
                    robotCentric   // Pedro uses "isRobotCentric"
            );
        }
    }

    public void toggleMode() {
        robotCentric = !robotCentric;
    }

    public CommandBuilder toggleCentric() {
        return Commands.instant(this::toggleMode);
    }

    public boolean isRobotCentric() {
        return robotCentric;
    }

    public void holdCurrent() {
        follower.holdPoint(new BezierPoint(follower.getPose()), follower.getHeading(), true);
        hold = true;
    }

    public void releaseHold() {
        hold = false;
    }

    public CommandBuilder hold() {
        return Commands.instant(this::holdCurrent);
    }

    public CommandBuilder release() {
        return Commands.instant(this::releaseHold);
    }

    public boolean isHolding() {
        return hold;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public double getT() {
        return follower.getCurrentTValue();
    }

    public Follower getFollower() {
        return follower;
    }

    // -------------------------------------------------------
    //                  RAW POSE RESET
    // -------------------------------------------------------

    /** Sets the robot’s raw Pose instantly (x, y, heading). */
    public void resetRawPose(Pose newPose) {
        follower.setPose(newPose);
    }

    /** CommandBuilder version for use in Ivy sequences. */
    public CommandBuilder resetTo(Pose pose) {
        return Commands.instant(() -> resetRawPose(pose));
    }
}