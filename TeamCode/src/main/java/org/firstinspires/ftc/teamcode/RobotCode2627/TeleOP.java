package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOP extends OpMode {
    double forward, strafe, turn,intakeVelocity,flywheelVelocityR, flywheelVelocityL, GateServoPos;
    init_class drive = new init_class();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }
/*THINGS TO STILL DO!!!
* Turret tracking and auto speed changing based on distance
* Turret Gate
* shooting controls
* flower intaker code
*/


    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        if (gamepad2.left_bumper){
            intakeVelocity = 1500;
        }else {
            intakeVelocity = 0;
        }
    }
}

