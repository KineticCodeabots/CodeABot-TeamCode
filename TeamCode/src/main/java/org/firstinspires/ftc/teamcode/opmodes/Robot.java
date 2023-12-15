package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Robot {
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double HEADING_THRESHOLD = 1.0;

    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    // Auto variables
    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;
    private IMU imu = null;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    static final double ARM_POWER = 0.3;
    private DcMotor armMotor = null;
    public int armTarget = 0;

    final private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public void init(boolean auto) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor = hardwareMap.get(DcMotor.class, "arm");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (auto) {
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        }
    }

    public void startAuto() {
        imu.resetYaw();
    }

    public void driveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed = turn;

        double left = drive + turn;
        double right = drive - turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        setDrivePower(left, right);
    }

    public void driveRobotAccel(double drive, double turn, int accelDistance, int currentPosition, int targetPosition) {
        driveSpeed = drive;
        turnSpeed = turn;

        // Calculate left and right wheel speeds
        double left = drive + turn;
        double right = drive - turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        // Calculate scaled speed based on currentPosition and targetPosition
        double scaledSpeed;
        if (currentPosition < accelDistance) {
            scaledSpeed = ((double) currentPosition / accelDistance);
        } else if (currentPosition > (targetPosition - accelDistance)) {
            scaledSpeed = ((double) (targetPosition - currentPosition) / accelDistance);
        } else {
            scaledSpeed = max;
        }

        // Apply scaled speed to left and right wheel speeds
        left *= scaledSpeed;
        right *= scaledSpeed;

        // Set the final drive power
        setDrivePower(left, right);
    }


    public void setDrivePower(double left, double right) {
        leftSpeed = left;
        rightSpeed = right;
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
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
            driveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                driveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendAutoTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            driveRobot(0, 0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
    public void sendDriveSpeedTelemetry() {
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftSpeed, rightSpeed);
    }

    void setArmTarget(int target) {
        armTarget = target;
        armMotor.setTargetPosition(armTarget);
        armMotor.setPower(ARM_POWER);
    }
}
