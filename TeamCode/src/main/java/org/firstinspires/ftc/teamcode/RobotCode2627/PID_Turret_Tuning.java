package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PID_Turret_Tuning extends OpMode {
    public DcMotorEx FlywheelLeft;
    public DcMotorEx FlywheelRight;

    double highVelocity = 2000;
       double lowVelocity = 500;

    double curTagetVelocity = highVelocity;
    double F = 0;
    double P = 0;

    double[] stepsizes = {0.1, 1, 10, 0.01, 0.001, 0.0001};

    int stepIndex = 1;



    @Override
    public void init() {
        FlywheelLeft = hardwareMap.get(DcMotorEx.class, "FlywheelMotorLeft");
        FlywheelRight = hardwareMap.get(DcMotorEx.class, "FlywheelMotorRight");
        FlywheelLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FlywheelRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FlywheelRight.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlywheelRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        FlywheelRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addData("Init Complete", ":)");
    }
    // 1. Declare this outside your loop (at the class level, or use a "holding" variable)
    boolean yLastState = false;
    @Override
    public void loop() {
   
    }
}
