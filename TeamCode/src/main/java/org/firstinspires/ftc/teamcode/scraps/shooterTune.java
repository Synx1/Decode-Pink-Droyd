package org.firstinspires.ftc.teamcode.scraps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.SubSystem.Shooter;

@TeleOp(name = "Shooter + Intake Test", group = "Test")
@Config
public class shooterTune extends OpMode {

    private Shooter shooter;
    private Intake intake;

    // This is what you'll change in FTC Dashboard (ticks/sec)
    public static double shooterTargetTicks = 0.0;

    // For R2 (right trigger) toggle edge detection
    private boolean lastRightTriggerPressed = false;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);

        shooter.off();          // start off
        intake.spinOff();       // intake off
    }

    @Override
    public void loop() {

        // ====== INTAKE CONTROL ======
        // R1 = right bumper, L1 = left bumper (hold-to-run style)
        if (gamepad1.right_bumper) {
            intake.spinIn();        // intake forward
        } else if (gamepad1.left_bumper) {
            intake.spinOut();       // intake reverse
        } else {
            intake.spinOff();       // stop intake when neither held
        }

        // ====== SHOOTER TOGGLE (R2 / right trigger) ======
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;

        if (rightTriggerPressed && !lastRightTriggerPressed) {
            shooter.shooterToggle();    // toggles activated flag and power
        }
        lastRightTriggerPressed = rightTriggerPressed;

        // ====== SHOOTER TARGET FROM DASHBOARD ======
        // shooterTargetTicks is set in FTC Dashboard
        shooter.setTarget(shooterTargetTicks);

        // Run PID+FF shooter control
        shooter.periodic();

        // ====== TELEMETRY ======
        telemetry.addData("Shooter Target (ticks/sec)", shooter.getTarget());
        telemetry.addData("Shooter Velocity (ticks/sec)", shooter.getVelocity());
        telemetry.addData("Shooter Error", shooter.getTarget() - shooter.getVelocity());
        telemetry.addData("Shooter At Speed", shooter.atTarget());
        telemetry.addData("Shooter Activated", shooter.isActivated());

        telemetry.addData("Intake Running", intake.isRunning());
        telemetry.addData("Intake State", intake.getState());
        telemetry.addData("Intake Current", intake.getTelemetry());

        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.off();
        intake.spinOff();
    }
}