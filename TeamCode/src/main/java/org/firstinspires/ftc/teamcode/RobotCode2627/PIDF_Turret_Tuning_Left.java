package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDF_Turret_Tuning_Left extends OpMode {
    public DcMotorEx FlywheelLeft;

    double highVelocity = 1500;
       double lowVelocity = 1000;

    double curTagetVelocity = highVelocity;
    double F = 0;
    double P = 0;

    double[] stepsizes = {0.1, 1, 10, 0.01, 0.001, 0.0001};

    int stepIndex = 1;



    @Override
    public void init() {
        FlywheelLeft = hardwareMap.get(DcMotorEx.class, "FlywheelMotorLeft");
        FlywheelLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlywheelLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addData("Init Complete", ":)");
    }
    private boolean yButtonWasDown = false;
    private boolean bButtonWasDown = false;
    private boolean DpadLeftButtonWasDown = false;
    private boolean DpadRightButtonWasDown = false;
    private boolean DpadUPButtonWasDown = false;
    private boolean DpadDownButtonWasDown;


    @Override
    public void loop() {
        // Inside your loop() method:
        boolean bButtonIsDown = gamepad1.b;
        boolean yButtonIsDown = gamepad1.y;
        boolean DpadLeftButtonIsDown = gamepad1.dpad_left;
        boolean DpadRightButtonIsDown = gamepad1.dpad_right;
        boolean DpadUpButtonIsDown = gamepad1.dpad_up;
        boolean DpadDownButtonIsDown = gamepad1.dpad_down;

        if (yButtonIsDown && !yButtonWasDown) {
            if (curTagetVelocity == highVelocity) {
                curTagetVelocity = lowVelocity;
            }else {curTagetVelocity = highVelocity;}
        }
        yButtonWasDown = yButtonIsDown;

        if (bButtonIsDown && !bButtonWasDown) {
            stepIndex = (stepIndex + 1) % stepsizes.length;
        }
        bButtonWasDown = bButtonIsDown;

        if (DpadLeftButtonIsDown && !DpadLeftButtonWasDown) {
            F -= stepsizes[stepIndex];
        }
        DpadLeftButtonWasDown = DpadLeftButtonIsDown;

        if (DpadLeftButtonIsDown && !DpadRightButtonWasDown) {
            F += stepsizes[stepIndex];
        }
        DpadRightButtonWasDown = DpadRightButtonIsDown;

        if (DpadUpButtonIsDown && !DpadUPButtonWasDown) {
            P += stepsizes[stepIndex];
        }
        DpadUPButtonWasDown = DpadUpButtonIsDown;

        if (DpadDownButtonIsDown && !DpadDownButtonWasDown) {
            P -= stepsizes[stepIndex];
        }
        DpadDownButtonWasDown = DpadDownButtonIsDown;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlywheelLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        FlywheelLeft.setVelocity(curTagetVelocity);
        double curVelocity = FlywheelLeft.getVelocity();
        double error = curTagetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTagetVelocity);
        telemetry.addData("Current Velocity", "X.2f", curVelocity);
        telemetry.addData("Error", "X.2f", error);
        telemetry.addLine("{---------------------------}");
        telemetry.addData("Tuning P Value", "X.4f (D-Pad U/D)", P );
        telemetry.addData("Tuning F Value", "X.4f (D-Pad R/L)", F );
        telemetry.addData("Step Size", "X.4f (B Botton)", stepsizes[stepIndex]);










    }
}
