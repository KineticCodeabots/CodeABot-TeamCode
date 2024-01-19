package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    // Motors
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public double driveSpeed = 0;
    public double turnSpeed = 0;
    public double leftSpeed = 0;
    public double rightSpeed = 0;

    public DcMotorEx armMotor = null;
    public ArmController arm = null;
    // Servos
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
        // Extract hardwareMap and telemetry from opMode
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    // Initialize hardware
    public void init() {
        // Ensure members are extracted as their not always(?) defined when constructor is executed.
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        // Initialize drive train
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Reset encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Initialize arm
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        arm = new ArmController(armMotor);
        arm.init();

        // Initialize servos
        handServo = hardwareMap.get(Servo.class, "hand");
        handServo.setDirection(Servo.Direction.REVERSE);

        gripperServo = hardwareMap.get(Servo.class, "gripper");
    }

    public void start() {

    }

    /**
     * Drive the robot with the given drive and turn speeds
     *
     * @param drive The speed to drive the robot forward/backward
     * @param turn  The speed to turn the robot left/right
     */
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

    /**
     * Set the power of the drive motors
     *
     * @param left
     * @param right
     */
    public void setDrivePower(double left, double right) {
        leftSpeed = left;
        rightSpeed = right;
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    /**
     * Does not call telemetry.update().
     */
    public void addTelemetry() {
        telemetry.addData("Drive", "left (%.2f), right (%.2f)", leftSpeed, rightSpeed);
        telemetry.addData("Arm", "power (%.2f)", armMotor.getPower());
        telemetry.addData("Hand", "position (%.2f), state (%s)", handPosition, getHandState());
        telemetry.addData("Gripper", "position (%.2f), open (%b)", gripperPosition, gripperOpen);
    }

    /**
     * Reset the servos to its default state
     */
    void resetServos() {
        setHandState(HandState.UP);
        setGripperState(true);
    }


    /**
     * Enum for the state of the hand servo
     */
    public enum HandState {
        DOWN,
        BACKDROP,
        UP
    }

    /**
     * Set the position of the hand servo
     *
     * @param state The state to set the hand servo to
     */
    void setHandState(HandState state) {
        switch (state) {
            case DOWN:
                setHandPosition(0.04);
                break;
            case BACKDROP:
                setHandPosition(0.25);
                break;
            case UP:
                setHandPosition(0.8);
                break;
        }
    }

    /**
     * Get the state of the hand servo
     *
     * @return The state of the hand servo
     */
    public HandState getHandState() {
        if (handPosition <= 0.05) {
            return HandState.DOWN;
        } else if (handPosition == 0.25) {
            return HandState.BACKDROP;
        } else if (handPosition == 0.8) {
            return HandState.UP;
        } else return null;
    }

    /**
     * Set the position of the hand servo
     *
     * @param position The position to set the hand servo to
     */
    void setHandPosition(double position) {
        handPosition = position;
        handServo.setPosition(position);
    }

    /**
     * Set the state of the gripper servo
     *
     * @param open Whether the gripper is opened or closed
     */
    void setGripperState(boolean open) {
        gripperOpen = open;
        if (open) {
            gripperPosition = 0.28;
        } else {
            gripperPosition = 0;
        }
        setGripperPosition(gripperPosition);
    }

    /**
     * Set the position of the gripper servo
     *
     * @param position The position to set the gripper servo to
     */
    void setGripperPosition(double position) {
        gripperPosition = position;
        gripperServo.setPosition(position);
    }
}
