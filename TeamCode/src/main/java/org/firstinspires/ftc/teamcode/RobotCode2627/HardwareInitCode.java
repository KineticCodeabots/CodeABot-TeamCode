package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareInitCode {
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor,turretMotor, intakeMotor;
    private DcMotorEx leftFlywheelMotor, rightFlywheelMotor;
    private IMU imu;

    private Servo turretGateServo, backPlateServo, leftFlowerRampServo, rightFlowerRampServo;

    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeft");
        backRightMotor = hwMap.get(DcMotor.class, "backRight");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

        leftFlywheelMotor = hwMap.get(DcMotorEx.class, "leftFlywheelMotor");
        rightFlywheelMotor = hwMap.get(DcMotorEx.class, "rightFlywheelMotor");
        turretMotor = hwMap.get(DcMotor.class, "turretMotor");

        turretGateServo = hwMap.get(Servo.class, "turretGateServo");
        backPlateServo = hwMap.get(Servo.class, "backPlateServo");
        leftFlowerRampServo = hwMap.get(Servo.class, "LFRServo");
        rightFlowerRampServo = hwMap.get(Servo.class, "RFRServo");

         frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
         backRightMotor.setDirection(DcMotor.Direction.REVERSE);
         leftFlywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);
         leftFlowerRampServo.setDirection(Servo.Direction.REVERSE);

         frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         leftFlywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
         rightFlywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

         imu = hwMap.get(IMU.class,"imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(RevOrientation));
    }
    public void drive(double forward, double strafe, double rotate, double intakePower) {

        double frontleftPower = forward + strafe + rotate;
        double frontrightPower = forward - strafe - rotate;
        double backleftPower = forward - strafe + rotate;
        double backrightPower = forward + strafe - rotate;


        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontleftPower));
        maxPower = Math.max(maxPower, Math.abs(frontrightPower));
        maxPower = Math.max(maxPower, Math.abs(backleftPower));
        maxPower = Math.max(maxPower, Math.abs(backrightPower));

        frontLeftMotor.setPower(maxSpeed * frontleftPower / maxPower);
        frontRightMotor.setPower(maxSpeed * frontrightPower / maxPower);
        backLeftMotor.setPower(maxSpeed * backleftPower / maxPower);
        backRightMotor.setPower(maxSpeed * backrightPower / maxPower);
        intakeMotor.setPower(intakePower);
    }
}
