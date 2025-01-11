package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoRobot {
    private final LinearOpMode opMode;
    private final Robot robot;

    public AutoRobot(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.robot = robot;
    }

    public void driveFieldCentric(double distance, double heading, double maxSpeed) {
        if (opMode.opModeIsActive()) {
            int moveCounts = (int) (distance * robot.COUNTS_PER_INCH);
            int frontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + moveCounts;
            int frontRightTarget = robot.frontRightMotor.getCurrentPosition() + moveCounts;
            int backLeftTarget = robot.backLeftMotor.getCurrentPosition() + moveCounts;
            int backRightTarget = robot.backRightMotor.getCurrentPosition() + moveCounts;

            robot.frontLeftMotor.setTargetPosition(frontLeftTarget);
            robot.frontRightMotor.setTargetPosition(frontRightTarget);
            robot.backLeftMotor.setTargetPosition(backLeftTarget);
            robot.backRightMotor.setTargetPosition(backRightTarget);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // motion profiling
            while (opMode.opModeIsActive() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {

            }

        }
    }

    public void turnAngle(double angle, double speed) {

    }
}
