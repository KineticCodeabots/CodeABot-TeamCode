package utils;

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
    List<String> motorNames = null;
    List<DcMotorEx> motors = new ArrayList<>();
    int index = 0;
    int lastIndex = 0;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        motorNames = new ArrayList<>(hardwareMap.getAllNames(DcMotorEx.class));
        for (String name : motorNames) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motors.add(motor);
            telemetry.addData("Motor", "\"%s\": position (%d)", name, motor.getCurrentPosition());
        }
        telemetry.update();
        waitForStart();
        DcMotorEx motor = motors.get(index);
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            telemetry.addData("Name", "\"%s\"", motorNames.get(index));
            telemetry.addData("Direction", "%s", motor.getDirection().toString());
            telemetry.addData("Run Mode", "%s", motor.getMode().toString());
            telemetry.addData("Zero Power Behavior", "%s", motor.getZeroPowerBehavior().toString());
            telemetry.addData("Power", "%.2f", motor.getPower());
            telemetry.addData("Position", "%d", motor.getCurrentPosition());
            telemetry.addData("Velocity", "%.2f", motor.getVelocity());
            telemetry.addData("Current", "%.2f", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            if (currentGamepad1.a && !previousGamepad1.a) {
                DcMotor.RunMode runMode = motor.getMode();
                if (runMode == DcMotor.RunMode.RUN_USING_ENCODER) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
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

            motor.setPower(-gamepad1.left_stick_y);

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                index--;
                if (index < 0) index = motors.size() - 1;
                motor = motors.get(index);
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                index++;
                if (index >= motors.size()) index = 0;
                motor = motors.get(index);
            }
            if (index != lastIndex) motor.setPower(0);

            lastIndex = index;
        }
    }
}
