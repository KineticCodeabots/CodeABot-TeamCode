package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Robot TeleOP", group = "Teleop")
public class TeleOP_MecanumDrive extends OpMode {
    BackRoundCode_MecanumDrive drive = new BackRoundCode_MecanumDrive();
    double forward, strafe, turn, intakePower, flywheelRPM, LeftServoPosition, RightServoPosition, BackPlatePower,  ActualFlywheelRPM;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }


    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        if (gamepad2.left_bumper){
            intakePower = 1;
        }
        else {
            intakePower = 0;
        }
        if (gamepad2.left_stick_button){
            intakePower = -0.2;
        }

        if (gamepad2.a){
            flywheelRPM = 1000;
        } else if (gamepad2.x) {
            flywheelRPM = 1250;
        } else if (gamepad2.y) {
            flywheelRPM = 1500;
        } else if (gamepad2.b) {
            flywheelRPM = 1750;
        }else if (gamepad2.right_bumper){
            flywheelRPM = 0;
        }

        if (gamepad2.dpad_down) {
            LeftServoPosition = 0;
            RightServoPosition = 0;
        }
        else if (gamepad2.dpad_up) {
            LeftServoPosition = 1;
            RightServoPosition = 1;
        }
        if (gamepad1.dpad_up){
            BackPlatePower = 0.5;
        } else if (gamepad1.dpad_down) {
            BackPlatePower = -0.5;
        }
        telemetry.addData("Flywheel RPM", ActualFlywheelRPM);
        if (LeftServoPosition == 0){
            telemetry.addData("Gate Position is ", "closed");
        } else if (LeftServoPosition == 1) {
            telemetry.addData("Gate Position is ", "open");
        }
        drive.drive(-forward, strafe, turn, flywheelRPM, intakePower, LeftServoPosition, RightServoPosition, BackPlatePower,  ActualFlywheelRPM);
    }
}
