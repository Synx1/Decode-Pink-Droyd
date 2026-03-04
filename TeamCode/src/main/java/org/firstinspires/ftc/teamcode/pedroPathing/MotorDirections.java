package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

@Disabled
@TeleOp(name = "Motor Directions", group = "Main")
public class MotorDirections extends OpMode {
    public static double leftFrontMotorDirection = 0;
    public static double leftBackMotorDirection = 0;
    public static double rightFrontMotorDirection = 0;
    public static double rightBackMotorDirection = 0;
    public static double test = 1;

    private MultipleTelemetry telemetryM;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private List<DcMotorEx> motors;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront.setDirection(direction(leftFrontMotorDirection));
        leftBack.setDirection(direction(leftBackMotorDirection));
        rightFront.setDirection(direction(rightFrontMotorDirection));
        rightBack.setDirection(direction(rightBackMotorDirection));

        motors = Arrays.asList(leftFront, leftBack, rightFront, rightBack);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryM = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        telemetryM.addLine("This will allow you to test the directions of your motors. You can change the directions in FTCDashboard -> FollowerConstants.");
        telemetryM.update();
    }

    @Override
    public void loop() {
        leftFront.setDirection(direction(leftFrontMotorDirection));
        leftBack.setDirection(direction(leftBackMotorDirection));
        rightFront.setDirection(direction(rightFrontMotorDirection));
        rightBack.setDirection(direction(rightBackMotorDirection));

        if(gamepad1.a)
            leftFront.setPower(1);
        else
            leftFront.setPower(0);

        if(gamepad1.y)
            leftBack.setPower(1);
        else
            leftBack.setPower(0);

        if(gamepad1.b)
            rightFront.setPower(1);
        else
            rightFront.setPower(0);

        if(gamepad1.x)
            rightBack.setPower(1);
        else
            rightBack.setPower(0);

        telemetryM.addLine("Press A to spin the left front motor at 100% power");
        telemetryM.addLine("Press Y to spin the left rear motor at 100% power");
        telemetryM.addLine("Press B to spin the right front motor at 100% power");
        telemetryM.addLine("Press X to spin the right rear motor at 100% power");
        telemetryM.addLine("Left Front Motor Direction: " + leftFrontMotorDirection);
        telemetryM.addLine("Left Rear Motor Direction: "+ leftBackMotorDirection);
        telemetryM.addLine("Right Front Motor Direction: "+ rightFrontMotorDirection);
        telemetryM.addLine("Right Rear Motor Direction: "+ rightBackMotorDirection);
        telemetryM.update();
    }

    public DcMotorSimple.Direction direction(double d) {
        if (d == 0)
            return DcMotorSimple.Direction.FORWARD;
        else
            return DcMotorSimple.Direction.REVERSE;
    }
}