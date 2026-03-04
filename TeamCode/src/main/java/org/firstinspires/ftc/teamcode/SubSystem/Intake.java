package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake {

    private final DcMotorEx motor;

    public static double OFF  = 0.0;
    public static double IDLE = 0.0;
    public static double IN   = 1.0;
    public static double OUT  = -1.0;

    public enum State {
        OFF,
        IDLE,
        IN,
        OUT
    }

    private State currentState = State.OFF;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "in");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        set(OFF);
    }

    /* ========================
       Core Control
       ======================== */

    public void set(double power) {
        motor.setPower(power);
    }

    public void spinIn() {
        set(IN);
        currentState = State.IN;
    }

    public void spinOut() {
        set(OUT);
        currentState = State.OUT;
    }

    public void spinIdle() {
        set(IDLE);
        currentState = State.IDLE;
    }

    public void spinOff() {
        set(OFF);
        currentState = State.OFF;
    }

    /* ========================
       Smart Toggle Logic
       ======================== */

    public void toggleIn() {
        if (currentState == State.IN) {
            spinOff();
        } else {
            spinIn();
        }
    }

    public void toggleOut() {
        if (currentState == State.OUT) {
            spinOff();
        } else {
            spinOut();
        }
    }

    /* ========================
       Command-Based Support
       ======================== */

    public CommandBuilder off() {
        return Commands.instant(this::spinOff);
    }

    public CommandBuilder in() {
        return Commands.instant(this::spinIn);
    }

    public CommandBuilder out() {
        return Commands.instant(this::spinOut);
    }

    public CommandBuilder idle() {
        return Commands.instant(this::spinIdle);
    }

    public CommandBuilder toggleInCommand() {
        return Commands.instant(this::toggleIn);
    }

    public CommandBuilder toggleOutCommand() {
        return Commands.instant(this::toggleOut);
    }

    /* ========================
       Telemetry
       ======================== */

    public boolean isRunning() {
        return currentState != State.OFF;
    }

    public State getState() {
        return currentState;
    }

    public String getTelemetry() {
        return String.format(
                "Intake: %s | %.2fA",
                currentState,
                motor.getCurrent(CurrentUnit.AMPS)
        );
    }
}