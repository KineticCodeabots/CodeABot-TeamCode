package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Servo Tester", group = "Testing")
@Disabled
public class ServoTester extends OpMode {
    String[] servoNames = null;
    Servo[] servos = null;
    Servo servo = null;
    int index = 0;
    int lastIndex = 0;

    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();


    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        servoNames = hardwareMap.getAllNames(Servo.class).toArray(new String[0]);
        servos = new Servo[servoNames.length];
        for (int i = 0; i < servoNames.length; i++) {
            String name = servoNames[i];
            Servo servo = hardwareMap.get(Servo.class, name);
            servos[i] = servo;
            telemetry.addData("Servo", "\"%s\": port (%d)", name, servo.getPortNumber());
        }
        telemetry.update();
        servo = servos[index];
    }

    @Override
    public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        telemetry.addData("Name", "\"%s\"", servoNames[index]);
        telemetry.addData("Port", "%d", servo.getPortNumber());
        telemetry.addData("Direction (Y)", "%s", servo.getDirection().toString());
        telemetry.addData("Position", "%.2f", servo.getPosition());
        telemetry.addData("Enabled", "%b", ((ServoImplEx) servo).isPwmEnabled());
        telemetry.update();

        if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
            servo.setPosition(servo.getPosition() - 0.05);
        } else if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
            servo.setPosition(servo.getPosition() + 0.05);
        }

        if (currentGamepad.cross && !previousGamepad.cross) {
            servo.setPosition(0);
        } else if (currentGamepad.circle && !previousGamepad.circle) {
            servo.setPosition(1);
        } else if (currentGamepad.square && !previousGamepad.square) {
            servo.setPosition(-1);
        }

        if (currentGamepad.options && !previousGamepad.options) {
            ServoImplEx servoEx = (ServoImplEx) servo;
            if (servoEx.isPwmEnabled()) {
                servoEx.setPwmDisable();
            } else {
                servoEx.setPwmEnable();
            }
        }

        if (currentGamepad.y && !previousGamepad.y) {
            Servo.Direction direction = servo.getDirection();
            if (direction == Servo.Direction.FORWARD) {
                servo.setDirection(Servo.Direction.REVERSE);
            } else {
                servo.setDirection(Servo.Direction.FORWARD);
            }
        }

        if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            index--;
            if (index < 0) index = servos.length - 1;
        } else if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
            index++;
            if (index >= servos.length) index = 0;
        }
        if (index != lastIndex) {
            servo = servos[index];
        }

        lastIndex = index;
    }
}
