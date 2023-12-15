package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "AutoTest", group = "Test")
public class AutoTest extends LinearOpMode {
    Robot robot = new Robot(this);
    double speed = 0.5;

    @Override
    public void runOpMode() {
        robot.init(true);
        waitForStart();
        robot.startAuto();
//        while (opModeIsActive()) {
//            robot.turnToHeading(speed, 90);
//            robot.turnToHeading(speed, 0);
//            sleep(1);
//        }

        double distance = 100;
//
        for (int i = 0; i < 5 && opModeIsActive(); i++) {
            robot.driveStraight(speed, distance, 0);
            robot.driveStraight(speed, -distance, 0);
        }
    }
}

