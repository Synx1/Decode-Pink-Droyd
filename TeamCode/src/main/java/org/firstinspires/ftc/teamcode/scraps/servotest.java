package org.firstinspires.ftc.teamcode.scraps;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Servo Adjust w/ Telemetry")
public class servotest extends LinearOpMode {

    private Servo servo0;

    // Step size
    private final double STEP = 0.05;

    @Override
    public void runOpMode() {

        servo0 = hardwareMap.get(Servo.class, "claw");

        // Start at current position or 0 if uninitialized
        double servoPos = servo0.getPosition();

        telemetry.addLine("Ready. R1 = +0.05, L1 = -0.05");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Raise position with R1
            if (gamepad1.right_bumper) {
                servoPos += STEP;
                sleep(150); // prevents spam increments
            }

            // Lower position with L1
            if (gamepad1.left_bumper) {
                servoPos -= STEP;
                sleep(150);
            }

            // Clip between 0 and 1
            servoPos = Math.max(0, Math.min(1, servoPos));

            // Apply to servo
            servo0.setPosition(servoPos);

            // Telemetry
            telemetry.addData("Servo Position", servoPos);
            telemetry.update();
        }
    }
}
