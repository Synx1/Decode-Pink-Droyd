package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

public class servo {
    private final Servo claw;

    private static final double OPEN_POSITION = 0.9;
    private static final double CLOSED_POSITION = 1.0;

    private boolean isOpen = false;

    public servo(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLOSED_POSITION);
    }

    public void periodic() {
    }

    public void open() {
        claw.setPosition(OPEN_POSITION);
        isOpen = true;
    }

    public void close() {
        claw.setPosition(CLOSED_POSITION);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) {
            close();
        } else {
            open();
        }
    }

    public String getTargetName() {
        return isOpen ? "OPEN" : "CLOSED";
    }

    public String getTelemetryString() {
        return String.format(Locale.US, "Claw: %s | Position: %.2f",
                getTargetName(), claw.getPosition());
    }
}