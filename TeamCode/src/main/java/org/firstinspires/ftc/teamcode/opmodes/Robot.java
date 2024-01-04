package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    // Hardware
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public double driveSpeed = 0;
    public double turnSpeed = 0;
    public double leftSpeed = 0;
    public double rightSpeed = 0;

    private DcMotor armMotor = null;

    public Servo handServo = null;
    public double handPosition = 0;

    public Servo gripperServo = null;
    public double gripperPosition = 0;
    public boolean gripperOpen = false;

    // OpMode
    final public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    // Initialize hardware
    public void init() {
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
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        handServo = hardwareMap.get(Servo.class, "hand");
        handServo.setDirection(Servo.Direction.REVERSE);

        gripperServo = hardwareMap.get(Servo.class, "gripper");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void start() {

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

    public void setDrivePower(double left, double right) {
        leftSpeed = left;
        rightSpeed = right;
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    /**
     * Does not call telemetry.update().
     */
    public void sendTelemetry() {
        telemetry.addData("Drive", "left (%.2f), right (%.2f)", leftSpeed, rightSpeed);
        telemetry.addData("Arm", "power (%.2f)", armMotor.getPower());
        telemetry.addData("Hand", "position (%.2f), state (%s)", handPosition, getHandState());
        telemetry.addData("Gripper", "position (%.2f), open (%b)", gripperPosition, gripperOpen);
    }

    void resetServos() {
        setHandState(HandState.UP);
        setGripperState(true);
    }

    void setArmPower(double power) {
        armMotor.setPower(power);
    }

    public enum HandState {
        DOWN,
        BACKBOARD,
        UP
    }
    void setHandState(HandState state) {
        switch (state) {
            case DOWN:
                setHandPosition(0);
                break;
            case BACKBOARD:
                setHandPosition(0.5);
                break;
            case UP:
                setHandPosition(1);
                break;
        }
    }
    public HandState getHandState() {
        if (handPosition == 0) {
            return HandState.DOWN;
        } else if (handPosition == 0.5) {
            return HandState.BACKBOARD;
        } else if (handPosition == 1) {
            return HandState.UP;
        } else return null;
    }

    void setHandPosition(double position) {
        handPosition = position;
        handServo.setPosition(position);
    }

    void setGripperState(boolean open) {
        gripperOpen = open;
        if (open) {
            gripperPosition = 0.5;
        } else {
            gripperPosition = 0;
        }
        setGripperPosition(gripperPosition);
    }
    void setGripperPosition(double position) {
        gripperPosition = position;
        gripperServo.setPosition(position);
    }
}
