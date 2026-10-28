package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp
public class SmallRobot extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor LeftMotor = hardwareMap.dcMotor.get("LeftMotor");
        DcMotor RightMotor = hardwareMap.dcMotor.get("RightMotor");

        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double maxSpeed = 1.0;
            double maxPower = 1.0;
            double y = gamepad1.right_trigger;
            double reverseY = gamepad1.left_trigger;
            double x = gamepad1.left_stick_x;

            // Calculate intended target power
            double targetLeftPower = (y - reverseY + x);
            double targetRightPower = (y - reverseY - x);

            maxPower = Math.max(maxPower, Math.abs(targetRightPower));
            maxPower = Math.max(maxPower, Math.abs(targetLeftPower));



            telemetry.update();
        }
    }
}