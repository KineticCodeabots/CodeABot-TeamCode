package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "TeleOp Mode")
public class TeleOpMode extends OpMode {
    DcMotor armMotor = null;
    CRServo armServo = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor leftBackMotor = null;
    IMU imu = null;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previosGamepad1 = new Gamepad();
    Gamepad previosGamepad2 = new Gamepad();


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
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
        armServo = hardwareMap.get(CRServo.class, "intakeServo");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

    }

    @Override
    public void loop() {


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }

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

        leftFrontMotor.setPower(frontLeftPower);
        leftBackMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightBackMotor.setPower(backRightPower);

        previosGamepad1.copy(currentGamepad1);
        previosGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        armMotor.setPower(gamepad1.left_stick_y);
        armServo.setPower(gamepad1.right_stick_y);
        telemetry.addData("Status", "Running");


    }
}
