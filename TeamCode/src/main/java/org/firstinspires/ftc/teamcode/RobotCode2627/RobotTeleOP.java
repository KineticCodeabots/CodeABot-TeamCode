package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.sql.Driver;

@TeleOp
public class RobotTeleOP extends OpMode {
    HardwareInitCode drive = new HardwareInitCode();
    double forward, strafe, rotate, intakePower;



    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        intakePower = gamepad1.right_trigger;
        drive.drive(forward, strafe, rotate, intakePower);

    }
}
