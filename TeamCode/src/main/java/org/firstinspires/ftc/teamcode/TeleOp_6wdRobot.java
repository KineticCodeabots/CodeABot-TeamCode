package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class TeleOp_6wdRobot extends OpMode {
    BackRoundCode6wdRobot drive = new BackRoundCode6wdRobot();
    double throttle, spin, intakePower;

    @Override
    public void init() {
        drive.init(hardwareMap); //initialises motor from Back_round_Code_6wd
    }

    @Override
    public void loop() { //sets throttle and spin to the according gamepad sticks and sets the intakes control to right trigger
        throttle = gamepad1.left_stick_y;
        spin = -gamepad1.right_stick_x;
        intakePower = gamepad1.right_trigger;

        drive.drive(throttle, spin, intakePower);

    }
}
