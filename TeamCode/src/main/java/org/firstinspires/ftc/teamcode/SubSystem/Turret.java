package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;


@Config
public class Turret {
    private final DcMotorEx motor;
    private PIDFController pidfController;

    public static double kP = 0.0045;
    public static double kI = 0.0;
    public static double kD = 0.0003;
    public static double kF = 0.0;

    public static double rpt = 0.0029919;

    public static double maxPower = .5;

    private double targetYaw = 0.0;
    private boolean isOn = true;
    private boolean manualMode = false;
    private double manualPower = 0.0;

    private int homePosition = 0;

    public Turret(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "TT");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        homePosition = motor.getCurrentPosition();
        targetYaw = 0.0;

        // Initialize PIDF controller
        pidfController = new PIDFController(kP, kI, kD, kF);
    }

    public void periodic() {
        if (!isOn) {
            motor.setPower(0);
            return;
        }

        if (manualMode) {
            motor.setPower(manualPower);
            return;
        }

        // Update PIDF coefficients from dashboard
        pidfController.setPIDF(kP, kI, kD, kF);

        // Target position in encoder ticks
        double targetPositionTicks = homePosition + (targetYaw / rpt);

        // Current position in encoder ticks
        int currentPositionTicks = motor.getCurrentPosition();

        // Calculate power using PIDF controller
        double power = pidfController.calculate(targetPositionTicks, currentPositionTicks);

        // Clamp power
        if (power > maxPower) power = maxPower;
        if (power < -maxPower) power = -maxPower;

        motor.setPower(power);
    }

    public void on() {
        isOn = true;
    }

    public void off() {
        isOn = false;
        motor.setPower(0);
    }

    public void manual(double power) {
        manualMode = true;
        manualPower = power;
        // Reset PIDF state when entering manual mode
        pidfController.reset();
    }

    public void automatic() {
        manualMode = false;
        // Reset PIDF state when entering automatic mode
        pidfController.reset();
    }

    public double getYaw() {
        return normalizeAngle((motor.getCurrentPosition() - homePosition) * rpt);
    }

    public void setYaw(double radians) {
        targetYaw = normalizeAngle(radians);
    }

    public void addYaw(double radians) {
        setYaw(getYaw() + radians);
    }

    public void face(Pose targetPose, Pose robotPose) {
        double angleToTarget = Math.atan2(
                targetPose.getY() - robotPose.getY(),
                targetPose.getX() - robotPose.getX()
        );
        double turretAngle = normalizeAngle(angleToTarget - robotPose.getHeading());
        setYaw(turretAngle);
    }

    public void resetTurret() {
        targetYaw = 0;
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2.0);
        if (angle <= -Math.PI) angle += Math.PI * 2.0;
        if (angle > Math.PI) angle -= Math.PI * 2.0;
        return angle;
    }

    public double getError() {
        return (homePosition + (targetYaw / rpt)) - motor.getCurrentPosition();
    }

    public boolean isReady() {
        return Math.abs(getError()) < 30;
    }

    public String getCurrent() {
        return String.format(Locale.US, "Turret: %.2f A", motor.getCurrent(CurrentUnit.AMPS));
    }

    public String getTelemetryString() {
        return String.format(Locale.US,
                "Yaw: %.1f° | Target: %.1f° | Error: %.0f ticks | Ready: %b",
                Math.toDegrees(getYaw()),
                Math.toDegrees(targetYaw),
                getError(),
                isReady()
        );
    }
}