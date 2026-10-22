package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Robot TeleOP", group = "Teleop")
public class TeleOP_MecanumDrive extends OpMode {
    BackRoundCode_MecanumDrive drive = new BackRoundCode_MecanumDrive();
    double forward, strafe, turn, intakePower, flywheelPowerL, flywheelPowerR, LeftServoPosition, RightServoPosition, BackPlatePower,  ActualFlywheelRPM;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }


    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        intakePower = gamepad2.right_trigger/2;

        if (gamepad2.a){
            flywheelPowerL = 1000;
            flywheelPowerR = 1000;
        } else if (gamepad2.x) {
            flywheelPowerL = 1250;
            flywheelPowerR = 1250;
        } else if (gamepad2.y) {
            flywheelPowerL = 1500;
            flywheelPowerR = 1500;
        } else if (gamepad2.b) {
            flywheelPowerL = 1750;
            flywheelPowerR = 1750;
        }else if (gamepad2.right_bumper){
            flywheelPowerL = 0;
            flywheelPowerR = 0;
        }

        if (gamepad2.dpad_left) {
            LeftServoPosition = 0;
            RightServoPosition = 0;
        }
        else if (gamepad2.dpad_right) {
            LeftServoPosition = 1;
            RightServoPosition = 1;
        }
        if (gamepad1.dpad_up){
            BackPlatePower = 0.5;
        } else{
            BackPlatePower = 0;
        }
        if (gamepad1.dpad_down){
            BackPlatePower = -0.5;
        }

        telemetry.addData("Flywheel RPM", ActualFlywheelRPM);
        if (LeftServoPosition == 0){
            telemetry.addData("Gate Position is ", "closed");
        } else if (LeftServoPosition == 1) {
            telemetry.addData("Gate Position is ", "open");
        }
        drive.drive(-forward, strafe, turn, flywheelPowerR , flywheelPowerL, intakePower, LeftServoPosition, RightServoPosition, BackPlatePower);
    }
}
