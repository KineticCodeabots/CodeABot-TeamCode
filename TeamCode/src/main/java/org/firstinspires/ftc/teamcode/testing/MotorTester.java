package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Motor Tester", group = "Testing")
public class MotorTester extends OpMode {
    String[] motorNames = null;
    DcMotorEx[] motors = null;
    DcMotorEx motor = null;
    int index = 0;
    int lastIndex = 0;

    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    boolean hold = false;

    @Override
    public void init() {
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
        motor = motors[index];
    }

    @Override
    public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        telemetry.addData("Name", "\"%s\"", motorNames[index]);
        telemetry.addData("Port", "%d", motor.getPortNumber());
        telemetry.addData("Direction (Y)", "%s", motor.getDirection().toString());
        telemetry.addData("Zero Power Behavior (A)", "%s", motor.getZeroPowerBehavior().toString());
        telemetry.addData("Run Mode (B)", "%s", motor.getMode().toString());
        telemetry.addData("Power", "%.2f", motor.getPower());
        telemetry.addData("Hold (A, DPad Up/Down)", "%b", hold);
        telemetry.addData("Position", "%d", motor.getCurrentPosition());
        telemetry.addData("Velocity", "%.2f", motor.getVelocity());
        telemetry.addData("Current", "%.2f", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();

        if (currentGamepad.b && !previousGamepad.b) {
            DcMotor.RunMode runMode = motor.getMode();
            if (runMode == DcMotor.RunMode.RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        if (currentGamepad.a && !previousGamepad.a) {
            DcMotor.ZeroPowerBehavior zeroPowerBehavior = motor.getZeroPowerBehavior();
            if (zeroPowerBehavior == DcMotor.ZeroPowerBehavior.BRAKE) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        if (currentGamepad.y && !previousGamepad.y) {
            DcMotorSimple.Direction direction = motor.getDirection();
            if (direction == DcMotorSimple.Direction.FORWARD) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        if (currentGamepad.x && !previousGamepad.x) {
            hold = !hold;
        }

        if (!hold) {
            motor.setPower(-gamepad1.left_stick_y);
        } else {
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                motor.setPower(motor.getPower() - 0.05);
            } else if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                motor.setPower(motor.getPower() + 0.05);
            }
        }

        if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            index--;
            if (index < 0) index = motors.length - 1;
        } else if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
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
