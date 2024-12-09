package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Arm Test", group = "Testing")
public class ArmTest extends OpMode {
    DcMotor armMotor = null;


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        if (currentGamepad1.cross && !previousGamepad1.cross) {
            armMotor.setTargetPosition(armMotor.getCurrentPosition() + 2000);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            telemetry.log().add("Setting target position to " + armMotor.getTargetPosition());
        }
        if (currentGamepad1.circle && !previousGamepad1.circle) {
            armMotor.setTargetPosition(armMotor.getCurrentPosition() - 2000);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            telemetry.log().add("Setting target position to " + armMotor.getTargetPosition());
        }
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
