package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MecanumDrive", group = "Teleop")
public class TeleOP_MecanumDrive extends OpMode {
    BackRoundCode_MecanumDrive drive = new BackRoundCode_MecanumDrive();
    double forward, strafe, turn;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        telemetry.addData("Left Front Power",forward + strafe + turn );
        telemetry.addData("Right Front Power",forward - strafe - turn );
        telemetry.addData("Left Back Power",forward - strafe + turn );
        telemetry.addData("Right Back Power",forward + strafe - turn);
        drive.drive(forward, strafe, turn);
    }
}
