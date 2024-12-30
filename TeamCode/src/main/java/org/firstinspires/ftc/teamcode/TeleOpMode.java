package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "TeleOp")
public class TeleOpMode extends OpMode {
    final double ARM_MAX_POWER = 0.5;
    final double LIFT_MAX_POWER = 0.5;
    final double MAX_DRIVE_SPEED = 0.3;

    DcMotor armMotor = null;
    DcMotor liftMotor = null;
    Servo claw = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor leftBackMotor = null;
    IMU imu = null;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previosGamepad1 = new Gamepad();
    Gamepad previosGamepad2 = new Gamepad();

    boolean clawOpened = true;


    @Override
    public void init() {
        // TODO: seperate robot class
        telemetry.addData("Status", "Initialized");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "clawServo");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

    }

    @Override
    public void loop() {
        previosGamepad1.copy(currentGamepad1);
        previosGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        update_drive();

        armMotor.setPower(gamepad2.left_stick_y * ARM_MAX_POWER);
        // TODO: preset arm positions
        // TODO: arm current warning
        // TODO: lift presets
        liftMotor.setPower(gamepad2.right_stick_y * LIFT_MAX_POWER);
        telemetry.addData("Status", "Running");
        if (currentGamepad2.cross && !previosGamepad2.cross) {
            if (clawOpened) {
                claw.setPosition(0.4);
                clawOpened = false;
            } else {
                claw.setPosition(0.1);
                clawOpened = true;
            }
        }
        telemetry.addData("Claw Position", claw.getPosition());
        // TODO: telemetry

    }

    void update_drive() {
        double y = -gamepad1.left_stick_y * MAX_DRIVE_SPEED; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1 * MAX_DRIVE_SPEED; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * MAX_DRIVE_SPEED;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontMotor.setPower(frontLeftPower);
        leftBackMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightBackMotor.setPower(backRightPower);
    }
}
