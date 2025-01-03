package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Robot {
    // TODO: better config system
    public static double ARM_MAX_POWER = 0.3;
    public static double LIFT_MAX_POWER = 0.5;
    public static double MAX_DRIVE_SPEED = 0.3;
    public static double MAX_TURN_SPEED = 0.3;
    public static double CLAW_OPEN_POSITION = 0.1;
    public static double CLAW_CLOSED_POSITION = 0.4;

    public DcMotorEx armMotor = null;
    public DcMotor liftMotor = null;
    public Servo claw = null;
    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor backLeftMotor = null;
    public IMU imu = null;

    public enum ClawState {
        OPEN,
        CLOSED
    }

    public ClawState clawState = ClawState.OPEN;

    // OpMode members
    OpMode opMode;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public Robot(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        // these members are only initialized at init stage
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.get(Servo.class, "clawServo");

        imu = hardwareMap.get(IMU.class, "imu");
        // TODO: configure imu correctly
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

    }

    void updateMecanumRobotDrive(double drive, double strafe, double turn) {
        double y = drive; // Remember, Y stick value is reversed
        double x = strafe * 1.1; // Counteract imperfect strafing
        double rx = turn;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    void updateMecanumFieldDrive(double drive, double strafe, double turn) {
        double y = drive * MAX_DRIVE_SPEED;
        double x = strafe * MAX_DRIVE_SPEED;
        double rx = turn * MAX_TURN_SPEED;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    void toggleClaw() {
        if (clawState == ClawState.OPEN) {
            claw.setPosition(CLAW_CLOSED_POSITION);
            clawState = ClawState.CLOSED;
        } else {
            claw.setPosition(CLAW_OPEN_POSITION);
            clawState = ClawState.OPEN;
        }
    }
}
