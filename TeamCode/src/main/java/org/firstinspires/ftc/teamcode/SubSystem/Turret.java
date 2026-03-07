package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

@Config
public class Turret {
    private final DcMotorEx motor;
    private final Follower follower;

    // ── Radians per tick ──
    public static double rpt = 0.0029919;

    // ── Coarse PIDF (large error, |error| > pidfSwitch) ──
    public static double kP = 0.003;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // ── Fine PIDF (settling, |error| <= pidfSwitch) ──
    public static double sP = 0.005;
    public static double sD = 0.0001;
    public static double sF = 0.0;

    // ── Tick threshold to switch controllers ──
    public static double pidfSwitch = 30;

    private PIDFController coarse;
    private PIDFController fine;

    private double targetTicks       = 0.0;
    private double homePositionTicks = 0.0;
    private double error             = 0.0;
    private double power             = 0.0;

    private boolean isOn        = true;
    private boolean isManual    = false;
    private double  manualPower = 0.0;

    public Turret(HardwareMap hardwareMap, Follower follower) {
        motor = hardwareMap.get(DcMotorEx.class, "TT");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);

        this.follower = follower;

        coarse = new PIDFController(new PIDFCoefficients(kP, 0, kD, kF));
        fine   = new PIDFController(new PIDFCoefficients(sP, 0, sD, sF));

        captureHomePosition();
    }

    public void periodic() {
        if (!isOn) {
            motor.setPower(0);
            return;
        }

        if (isManual) {
            motor.setPower(manualPower);
            return;
        }

        // Re-apply gains each loop so Dashboard changes take effect
        coarse.setCoefficients(new PIDFCoefficients(kP, 0, kD, kF));
        fine.setCoefficients(new PIDFCoefficients(sP, 0, sD, sF));

        error = targetTicks - motor.getCurrentPosition();

        if (Math.abs(error) > pidfSwitch) {
            coarse.updateError(error);
            coarse.updateFeedForwardInput(Math.signum(error));
            power = coarse.run();
        } else {
            fine.updateError(error);
            power = fine.run();
        }

        motor.setPower(power);
    }

    // ── Yaw helpers ──

    public double getYaw() {
        return normalizeAngle((motor.getCurrentPosition() - homePositionTicks) * rpt);
    }

    /**
     * Sets target yaw relative to saved home position.
     * 0 rad = init home position.
     */
    public void setYaw(double radians) {
        targetTicks = homePositionTicks + (normalizeAngle(radians) / rpt);
    }

    public void addYaw(double radians) {
        setYaw(getYaw() + radians);
    }

    // ── Home position ──

    /**
     * Captures the turret's CURRENT encoder position as home.
     * Call this during init if you want the current physical angle to be the return point.
     */
    public void captureHomePosition() {
        homePositionTicks = motor.getCurrentPosition();
        targetTicks = homePositionTicks;
    }

    /** Returns turret to the encoder position saved as home. */
    public void goHome() {
        targetTicks = homePositionTicks;
    }

    public double getHomePositionTicks() {
        return homePositionTicks;
    }

    // ── Goal tracking ──

    /**
     * Points turret at targetPose.
     * Reads robot pose + heading from Pedro Follower internally.
     */
    public void face(Pose targetPose) {
        face(targetPose, follower.getPose());
    }

    /**
     * Points turret at targetPose given an explicit robotPose.
     * Uses MathUtilities.faceAngle for centralized math.
     */
    public void face(Pose targetPose, Pose robotPose) {
        setYaw(MathUtilities.faceAngle(robotPose, targetPose));
    }

    // ── Manual control ──

    public void manual(double power) {
        isManual    = true;
        manualPower = power;
    }

    public void automatic() {
        isManual = false;
        manualPower = 0.0;
    }

    // ── On / Off ──

    public void on() {
        isOn = true;
    }

    public void off() {
        isOn = false;
        motor.setPower(0);
    }

    // ── Reset ──

    public void resetTurret() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        captureHomePosition();
    }

    // ── Utilities ──

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2.0);
        if (angle <= -Math.PI) angle += Math.PI * 2.0;
        if (angle > Math.PI) angle -= Math.PI * 2.0;
        return angle;
    }

    public double getError() {
        return error;
    }

    public double getTarget() {
        return targetTicks;
    }

    public boolean isReady() {
        return Math.abs(error) < 30;
    }

    public String getCurrent() {
        return String.format(Locale.US, "Turret: %.2f A", motor.getCurrent(CurrentUnit.AMPS));
    }

    public String getTelemetryString() {
        return String.format(
                Locale.US,
                "Yaw: %.1f° | Target: %.1f° | Home: %.0f | Error: %.0f ticks | Ready: %b | Mode: %s",
                Math.toDegrees(getYaw()),
                Math.toDegrees((targetTicks - homePositionTicks) * rpt),
                homePositionTicks,
                error,
                isReady(),
                isManual ? "MANUAL" : (Math.abs(error) > pidfSwitch ? "COARSE" : "FINE")
        );
    }
}