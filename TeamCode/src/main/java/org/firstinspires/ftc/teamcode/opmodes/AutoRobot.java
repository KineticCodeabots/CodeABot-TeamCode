package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AutoRobot extends Robot {
    // Constants for motor encoder counts per revolution
    static final double COUNTS_PER_MOTOR_REV = 1120;
    // Gear reduction applied to the motor's output
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    // Diameter of the wheels in inches
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    // Calculate the number of encoder counts per inch traveled
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI); // Formula based on motor specs and wheel size

    // Tolerated degrees to target heading
    static final double HEADING_THRESHOLD = 1.0;
    // Proportional speed constant for turning in different situations
    // TODO: tune maybe?
    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;
    // Distance to accelerate and decelerate
    static final int ACCEL_DISTANCE_GAIN = (int) (10 * COUNTS_PER_INCH);
    static final int DECEL_DISTANCE_GAIN = (int) (20 * COUNTS_PER_INCH);
    // Minimum speed when accelerating or decelerating
    static final double MIN_ACCEL_SPEED = 0.2;
    static final double MIN_DECEL_SPEED = 0.05;
    // Global members for telemetry
    private double headingError = 0;
    private double targetHeading = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;
    // Interial Measurement Unit
    private IMU imu = null;

    public AutoRobot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {
        super.init();
        // Override zero power behavior to brake
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Initialize IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void start() {
        super.start();
        imu.resetYaw(); // Resets heading to 0
    }
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (!opMode.opModeIsActive()) return;

        // Determine new target position, and pass to motor controller
        int accelDistance = (int) (ACCEL_DISTANCE_GAIN * maxDriveSpeed);
        int decelDistance = (int) (DECEL_DISTANCE_GAIN * maxDriveSpeed);
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        int targetPosition = Math.abs(moveCounts);
        int leftStart = leftDrive.getCurrentPosition();
        int rightStart = rightDrive.getCurrentPosition();
        leftTarget = leftDrive.getCurrentPosition() + moveCounts;
        rightTarget = rightDrive.getCurrentPosition() + moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        driveRobot(Math.min(MIN_ACCEL_SPEED, maxDriveSpeed), 0);

        // keep looping while we are still active, and BOTH motors are running.
        while (opMode.opModeIsActive() &&
                (leftDrive.isBusy() && rightDrive.isBusy())) {
            // Determine current distance traveled
            int currentPosition = Math.max(Math.abs(leftDrive.getCurrentPosition() - leftStart), Math.abs(rightDrive.getCurrentPosition() - rightStart));
            double accelSpeed;
            // Accelerate or decelerate based on distance to target
            if (currentPosition < accelDistance) {
                accelSpeed = Math.max(((double) currentPosition / accelDistance) * maxDriveSpeed, MIN_ACCEL_SPEED);
            } else if (currentPosition > (targetPosition - decelDistance)) {
                accelSpeed = Math.max((((double) targetPosition - currentPosition) / decelDistance) * maxDriveSpeed, MIN_DECEL_SPEED);
            } else {
                accelSpeed = maxDriveSpeed;
            }

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Chooes min value between accelSpeed and maxDriveSpeed for driving speed and apply the turning correction to the current driving speed.
            driveRobot(Math.min(accelSpeed, maxDriveSpeed), turnSpeed);

            // Display drive status for the driver.
            sendAutoTelemetry(true);
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        driveRobot(0, 0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            driveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendAutoTelemetry(false);
        }

        // Stop all motion;
        driveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            double turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            driveRobot(0, turnSpeed);
        }

        // Stop all motion;
        driveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
        // Determine the heading current error
        headingError = targetHeading - getHeading();
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES) * -1;
    }


    private void sendAutoTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }
}
