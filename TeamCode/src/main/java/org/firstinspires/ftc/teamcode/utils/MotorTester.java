package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Motor Tester", group = "Utils")
public class MotorTester extends LinearOpMode {
    String[] motorNames = null;
    DcMotorEx[] motors = null;
    int index = 0;
    int lastIndex = 0;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    boolean hold = false;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        motorNames = hardwareMap.getAllNames(DcMotorEx.class).toArray(new String[0]);
        motors = new DcMotorEx[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            String name = motorNames[i];
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motors[i] = motor;
            telemetry.addData("Motor", "\"%s\": position (%d)", name, motor.getCurrentPosition());
        }
        telemetry.update();
        waitForStart();
        DcMotorEx motor = motors[index];
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            telemetry.addData("Name:", "\"%s\"", motorNames[index]);
            telemetry.addData("Port:", "%d", motor.getPortNumber());
            telemetry.addData("Direction (Y):", "%s", motor.getDirection().toString());
            telemetry.addData("Zero Power Behavior (A):", "%s", motor.getZeroPowerBehavior().toString());
            telemetry.addData("Run Mode (B):", "%s", motor.getMode().toString());
            telemetry.addData(hold ? "Power (HOLD, Dpad Up/Down):" : "Power:", "%.2f", motor.getPower());
            telemetry.addData("Position:", "%d", motor.getCurrentPosition());
            telemetry.addData("Velocity:", "%.2f", motor.getVelocity());
            telemetry.addData("Current:", "%.2f", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            if (currentGamepad1.b && !previousGamepad1.b) {
                DcMotor.RunMode runMode = motor.getMode();
                if (runMode == DcMotor.RunMode.RUN_USING_ENCODER) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                DcMotor.ZeroPowerBehavior zeroPowerBehavior = motor.getZeroPowerBehavior();
                if (zeroPowerBehavior == DcMotor.ZeroPowerBehavior.BRAKE) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                DcMotorSimple.Direction direction = motor.getDirection();
                if (direction == DcMotorSimple.Direction.FORWARD) {
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                hold = !hold;
            }

            if (!hold) {
                motor.setPower(-gamepad1.left_stick_y);
            } else {
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    motor.setPower(motor.getPower() - 0.05);
                } else if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    motor.setPower(motor.getPower() + 0.05);
                }
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                index--;
                if (index < 0) index = motors.length - 1;
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                index++;
                if (index >= motors.length) index = 0;
            }
            if (index != lastIndex) {
                motor.setPower(0);
                motor = motors[index];
            }

            lastIndex = index;
        }
    }
}
