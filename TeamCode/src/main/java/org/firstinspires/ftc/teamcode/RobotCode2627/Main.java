package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Main extends OpMode {
    private DcMotorEx FlywheelLeft;
    private DcMotorEx FlywheelRight;
    private DcMotorEx RotationMotor;
    private Servo BPServo;

    private boolean toggle_flywheel = false;

    public void handle_gamepad(){
        if (gamepad1.a) {
            toggle_flywheel = !toggle_flywheel;
        }
    }

    public void handle_flywheel(){
        if (toggle_flywheel) {
            FlywheelLeft.setVelocity(1000);
            FlywheelRight.setVelocity(1000);
        }
    }

    @Override
    public void init() {
        FlywheelLeft = hardwareMap.get(DcMotorEx.class, "FlywheelMotorLeft");
        FlywheelRight = hardwareMap.get(DcMotorEx.class, "FlywheelMotorRight");
        RotationMotor = hardwareMap.get(DcMotorEx.class, "TurretRotationMotor");
        BPServo = hardwareMap.get(Servo.class, "BackPL ateServo");
        FlywheelLeft.setDirection(DcMotorEx.Direction.REVERSE);
        FlywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Joel, Welcome to the jungle!"); //if confused by random telemetry, it is a inside joke between Joel + Miles 
    }

    @Override
    public void loop() {
        handle_gamepad();
        handle_flywheel();
    }
}
