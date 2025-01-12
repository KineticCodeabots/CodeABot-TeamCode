package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class AutoRobot {
    public static double COUNTS_PER_INCH = 1;

    private final LinearOpMode opMode;
    private final Robot robot;


    private final PID drivePIDController = new PID(0.1, 0.01, 0.05); // TODO: tune

    public AutoRobot(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.robot = robot;
    }

    public void drive(double distance, double maxAcceleration, double maxVelocity) {
        if (opMode.opModeIsActive()) {
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            int frontLeftStartPosition = robot.frontLeftMotor.getCurrentPosition();
            int frontRightStartPosition = robot.frontRightMotor.getCurrentPosition();
            int backLeftStartPosition = robot.backLeftMotor.getCurrentPosition();
            int backRightStartPosition = robot.backRightMotor.getCurrentPosition();

            int frontLeftTarget = frontLeftStartPosition + moveCounts;
            int frontRightTarget = frontRightStartPosition + moveCounts;
            int backLeftTarget = backLeftStartPosition + moveCounts;
            int backRightTarget = backRightStartPosition + moveCounts;

            robot.frontLeftMotor.setTargetPosition(frontLeftTarget);
            robot.frontRightMotor.setTargetPosition(frontRightTarget);
            robot.backLeftMotor.setTargetPosition(backLeftTarget);
            robot.backRightMotor.setTargetPosition(backRightTarget);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Initialize motion profile parameters
            double elapsedTime = 0;

            ElapsedTime runtime = new ElapsedTime();

            while (opMode.opModeIsActive() && robot.frontLeftMotor.isBusy() &&
                    robot.frontRightMotor.isBusy() &&
                    robot.backLeftMotor.isBusy() &&
                    robot.backRightMotor.isBusy()) {

                elapsedTime = runtime.seconds();

                double targetPosition = motionProfile(maxAcceleration, maxVelocity, distance, elapsedTime);

                // Use PID controller to calculate motor power
                double currentPosition = robot.frontLeftMotor.getCurrentPosition();
                double motorPower = drivePIDController.update(targetPosition, currentPosition);

                // Apply motor power
                robot.frontLeftMotor.setPower(motorPower);
                robot.frontRightMotor.setPower(motorPower);
                robot.backLeftMotor.setPower(motorPower);
                robot.backRightMotor.setPower(motorPower);

                opMode.telemetry.addData("Target Position", targetPosition);
                opMode.telemetry.addData("Current Position", currentPosition);
                opMode.telemetry.addData("Motor Power", motorPower);
                opMode.telemetry.update();
            }

            // Stop motors
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // Trapezoidal motion profile method
    private double motionProfile(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {
        double accelerationTime = maxVelocity / maxAcceleration;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);

        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseTime = cruiseDistance / maxVelocity;
        double totalTime = 2 * accelerationTime + cruiseTime;

        if (elapsedTime > totalTime) {
            return distance; // Completed profile
        } else if (elapsedTime < accelerationTime) {
            return 0.5 * maxAcceleration * Math.pow(elapsedTime, 2); // Accelerating
        } else if (elapsedTime < accelerationTime + cruiseTime) {
            return accelerationDistance + maxVelocity * (elapsedTime - accelerationTime); // Cruising
        } else {
            double decelerationTime = elapsedTime - accelerationTime - cruiseTime;
            return accelerationDistance + cruiseDistance +
                    maxVelocity * decelerationTime -
                    0.5 * maxAcceleration * Math.pow(decelerationTime, 2); // Decelerating
        }
    }
}
