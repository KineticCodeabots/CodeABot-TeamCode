package org.firstinspires.ftc.teamcode.opmodes.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Motor Tester", group = "Utils")
public class MotorTester extends LinearOpMode {
    List<String> motorNames = null;
    List<DcMotorEx> motors = new ArrayList<DcMotorEx>();
    int index = 0;
    int lastIndex = 0;

    @Override
    public void runOpMode() {
        motorNames = new ArrayList<String>(hardwareMap.getAllNames(DcMotorEx.class));
        for (String name : motorNames) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motors.add(motor);
            telemetry.addData("Motor", "\"%s\": position (%d)", name, motor.getCurrentPosition());
        }
        telemetry.update();
        waitForStart();
        DcMotorEx motor = motors.get(index);
        while (opModeIsActive()) {
            telemetry.addData("Name", "\"%s\"", motorNames.get(index));
            telemetry.addData("Power", "%.2f", motor.getPower());
            telemetry.addData("Position", "%d", motor.getCurrentPosition());
            telemetry.addData("Velocity", "%.2f", motor.getVelocity());
            telemetry.addData("Run Mode", "%s", motor.getMode().toString());
            telemetry.addData("Zero Power Behavior", "%s", motor.getZeroPowerBehavior().toString());

            if (gamepad1.a) {
                DcMotor.RunMode runMode = motor.getMode();
                if (runMode == DcMotor.RunMode.RUN_USING_ENCODER) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            if (gamepad1.b) {
                DcMotor.ZeroPowerBehavior zeroPowerBehavior = motor.getZeroPowerBehavior();
                if (zeroPowerBehavior == DcMotor.ZeroPowerBehavior.BRAKE) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }

            motor.setPower(-gamepad1.left_stick_y);

            if (gamepad1.left_bumper) {
                index--;
                if (index < 0) index = motors.size() - 1;
                motor = motors.get(index);
            } else if (gamepad1.right_bumper) {
                index++;
                if (index >= motors.size()) index = 0;
                motor = motors.get(index);
            }
            if (index != lastIndex) motor.setPower(0);

            lastIndex = index;
        }
    }
}
