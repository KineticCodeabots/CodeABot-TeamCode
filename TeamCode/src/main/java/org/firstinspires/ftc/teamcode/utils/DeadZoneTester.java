package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DeadZoneTester extends OpMode {
    double leftStickXMin = Double.NaN;
    double leftStickYMin = Double.NaN;
    double rightStickXMin = Double.NaN;
    double rightStickYMin = Double.NaN;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double leftStickX = Math.abs(gamepad1.left_stick_x);
        double leftStickY = Math.abs(gamepad1.left_stick_y);
        double rightStickX = Math.abs(gamepad1.right_stick_x);
        double rightStickY = Math.abs(gamepad1.right_stick_y);

        if (leftStickX != 0 && (leftStickX < leftStickXMin || Double.isNaN(leftStickXMin))) {
            leftStickXMin = leftStickX;
        }
        if (leftStickY != 0 && (leftStickY < leftStickYMin || Double.isNaN(leftStickYMin))) {
            leftStickYMin = leftStickY;
        }
        if (rightStickX != 0 && (rightStickX < rightStickXMin || Double.isNaN(rightStickXMin))) {
            rightStickXMin = rightStickX;
        }
        if (rightStickY != 0 && (rightStickY < rightStickYMin || Double.isNaN(rightStickYMin))) {
            rightStickYMin = rightStickY;
        }

        telemetry.addData("leftStickX", leftStickX);
        telemetry.addData("leftStickY", leftStickY);
        telemetry.addData("rightStickX", rightStickX);
        telemetry.addData("rightStickY", rightStickY);

        telemetry.addData("leftStickXMin", leftStickXMin);
        telemetry.addData("leftStickYMin", leftStickYMin);
        telemetry.addData("rightStickXMin", rightStickXMin);
        telemetry.addData("rightStickYMin", rightStickYMin);


        telemetry.update();
    }
}
