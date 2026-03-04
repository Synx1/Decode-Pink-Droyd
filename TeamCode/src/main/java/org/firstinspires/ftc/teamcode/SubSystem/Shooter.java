package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
public class Shooter {

    // Flywheel motors
    private DcMotorEx S1, S2;

    // Claw servo instead of gate/hood
    private final Servo claw;

    // Target velocity (ticks/sec)
    private double t = 0;

    // Feedforward + P gains (tunable in Dashboard)
    public static double kS = 0.08;
    public static double kV = 0.0005;
    public static double kP = 0.02;

    // Simple preset velocities (you can tune)
    public static double near = 1150;
    public static double far  = 1420;
    public static double shootstop = 0;

    // At-speed tolerance
    public static double VELOCITY_TOLERANCE = 50;

    // Safety cap
    public static double MAX_TICKS_PER_SEC = 1500;

    // Claw positions
    public static double OPEN_POSITION   =.9;
    public static double CLOSED_POSITION = 0;

    private boolean activated  = true;
    private boolean clawClosed = true;

    // Your quadratic regression: distance (in) -> ticks/sec
    // y = 0.0556251x^2 - 5.95884x + 1054.63079
    public static double regA = 0.0394752;
    public static double regB = -5.75992;
    public static double regC = 1454.17638;

    public Shooter(HardwareMap hardwareMap) {
        S1 = hardwareMap.get(DcMotorEx.class, "S1");
        S2 = hardwareMap.get(DcMotorEx.class, "S2");
        claw = hardwareMap.get(Servo.class, "claw");

        // Reverse one side so both spin the same physical direction
        S1.setDirection(DcMotorSimple.Direction.REVERSE);
        S2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Custom control using power → use RUN_WITHOUT_ENCODER
        S1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        S2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        S1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        S2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        clawClose();
    }

    /* ========================
       Basic getters
       ======================== */

    public double getTarget() {
        return t;
    }

    /** Average velocity of both shooters in ticks/sec. */
    public double getVelocity() {
        return (Math.abs(S1.getVelocity()) + Math.abs(S2.getVelocity())) / 2.0;
    }

    public double getS1Velocity() {
        return S1.getVelocity();
    }

    public double getS2Velocity() {
        return S2.getVelocity();
    }

    /* ========================
       Power + activation
       ======================== */

    private void setPower(double p) {
        // Clamp to [0, 1] because shooter should only spin forward
        p = Math.max(0.0, Math.min(1.0, p));
        S1.setPower(p);
        S2.setPower(p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public void shooterToggle() {
        activated = !activated;
        if (!activated) {
            setPower(0);
        }
    }

    public CommandBuilder toggle() {
        return Commands.instant(this::shooterToggle);
    }

    /* ========================
       Presets (near / far)
       ======================== */

    public void shootFar() {
        setTarget(far);
        on();
    }

    public void shootNear() {
        setTarget(near);
        on();
    }


    public void shootstop() {
        setTarget(shootstop);
        on();
    }

    public CommandBuilder stop() {
        return Commands.instant(this::shootstop);
    }

    public CommandBuilder near() {
        return Commands.instant(this::shootNear);
    }

    public CommandBuilder farCommand() {
        return Commands.instant(this::shootFar);
    }

    /* ========================
       Target control
       ======================== */

    public void setTarget(double velocityTicksPerSec) {
        // Cap at safe max
        t = Math.max(0, Math.min(velocityTicksPerSec, MAX_TICKS_PER_SEC));
    }

    public void setTargetFromDistance(double distanceInches) {
        double v = regA * distanceInches * distanceInches
                + regB * distanceInches
                + regC;
        setTarget(v);
    }

    public void setTargetFromPose(Pose current, Pose goal) {
        double dx = goal.getX() - current.getX();
        double dy = goal.getY() - current.getY();
        double distance = Math.hypot(dx, dy);
        setTargetFromDistance(distance);
    }

    /* ========================
       Periodic update (PID + FF)
       ======================== */

    public void periodic() {
        if (!activated || t <= 0) {
            setPower(0);
            return;
        }

        double currentVelocity = getVelocity();
        double error = t - currentVelocity;

        double power = (kV * t) + (kP * error) + kS;
        setPower(power);
    }

    public boolean atTarget() {
        return t > 0 && Math.abs(t - getVelocity()) < VELOCITY_TOLERANCE;
    }

    /* ========================
       Claw handling
       ======================== */

    public void clawOpen() {
        claw.setPosition(OPEN_POSITION);
        clawClosed = false;
    }

    public void clawClose() {
        claw.setPosition(CLOSED_POSITION);
        clawClosed = true;
    }

    public void toggleClaw() {
        if (clawClosed) {
            clawOpen();
        } else {
            clawClose();
        }
    }

    /* ========================
       Telemetry helpers
       ======================== */

    public String getLeftCurrent() {
        return "Left Shooter Motor: " + S1.getCurrent(CurrentUnit.AMPS);
    }

    public String getRightCurrent() {
        return "Right Shooter Motor: " + S2.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isActivated() {
        return activated;
    }

    public boolean isClawClosed() {
        return clawClosed;
    }
}