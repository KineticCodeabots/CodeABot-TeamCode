package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TeleOp Mode")
public class TeleOpMode extends OpMode {
    DcMotor armMotor = null;
    CRServo armServo = null;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previosGamepad1 = new Gamepad();
    Gamepad previosGamepad2 = new Gamepad();


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armServo = hardwareMap.get(CRServo.class, "intakeServo");

    }

    @Override
    public void loop() {
        previosGamepad1.copy(currentGamepad1);
        previosGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        armMotor.setPower(gamepad1.left_stick_y);
        armServo.setPower(gamepad1.right_stick_y);
        telemetry.addData("Status", "Running");


    }
}
