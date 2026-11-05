package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDF_Turret_Tuning_Right extends OpMode {

    public DcMotorEx FlywheelRight;

    double highVelocity = 1500;
    double lowVelocity = 1000;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepsizes = {0.1, 1, 10, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    // Motor start/stop
    boolean motorEnabled = false;

    // Velocity accuracy measurements
    double totalAbsoluteError = 0;
    int errorSamples = 0;
    double statsStartTime = 0;

    double velocityTolerance = 15;

    // Your original button edge detection
    private boolean yButtonWasDown = false;
    private boolean bButtonWasDown = false;
    private boolean aButtonWasDown = false;

    private boolean DpadLeftButtonWasDown = false;
    private boolean DpadRightButtonWasDown = false;
    private boolean DpadUPButtonWasDown = false;
    private boolean DpadDownButtonWasDown = false;
    @Override
    public void init() {
        FlywheelRight = hardwareMap.get(
                DcMotorEx.class, "FlywheelMotorRight");

        FlywheelRight.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        FlywheelRight.setDirection(
                DcMotorSimple.Direction.REVERSE);

        // Start from the motor's configured PIDF values
        PIDFCoefficients initial =
                FlywheelRight.getPIDFCoefficients(
                        DcMotor.RunMode.RUN_USING_ENCODER);

        P = initial.p;
        F = initial.f;

        applyPIDF();

        FlywheelRight.setPower(0);

        resetStats();

        telemetry.addData("Init Complete", ":)");
        telemetry.update();
    }

    private void applyPIDF() {

        PIDFCoefficients pidfCoefficients =
                new PIDFCoefficients(P, 0, 0, F);

        FlywheelRight.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients);
    }

    private void resetStats() {
        totalAbsoluteError = 0;
        errorSamples = 0;
        statsStartTime = getRuntime();
    }

    @Override
    public void loop() {

        boolean bButtonIsDown = gamepad1.b;
        boolean yButtonIsDown = gamepad1.y;
        boolean aButtonIsDown = gamepad1.a;

        boolean DpadLeftButtonIsDown = gamepad1.dpad_left;
        boolean DpadRightButtonIsDown = gamepad1.dpad_right;
        boolean DpadUpButtonIsDown = gamepad1.dpad_up;
        boolean DpadDownButtonIsDown = gamepad1.dpad_down;

        boolean pidfChanged = false;

        // Y: Toggle target velocity
        if (yButtonIsDown && !yButtonWasDown) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
            resetStats();
        }
        yButtonWasDown = yButtonIsDown;

        // B: Change tuning step size
        if (bButtonIsDown && !bButtonWasDown) {
            stepIndex = (stepIndex + 1) % stepsizes.length;
        }
        bButtonWasDown = bButtonIsDown;

        // A: Start/stop flywheel
        if (aButtonIsDown && !aButtonWasDown) {
            motorEnabled = !motorEnabled;
            resetStats();
        }
        aButtonWasDown = aButtonIsDown;

        // D-pad LEFT: Decrease F
        if (DpadLeftButtonIsDown && !DpadLeftButtonWasDown) {
            F = Math.max(0, F - stepsizes[stepIndex]);
            pidfChanged = true;
        }
        DpadLeftButtonWasDown = DpadLeftButtonIsDown;

        // D-pad RIGHT: Increase F
        if (DpadRightButtonIsDown && !DpadRightButtonWasDown) {
            F += stepsizes[stepIndex];
            pidfChanged = true;
        }
        DpadRightButtonWasDown = DpadRightButtonIsDown;

        // D-pad UP: Increase P
        if (DpadUpButtonIsDown && !DpadUPButtonWasDown) {
            P += stepsizes[stepIndex];
            pidfChanged = true;
        }
        DpadUPButtonWasDown = DpadUpButtonIsDown;

        // D-pad DOWN: Decrease P
        if (DpadDownButtonIsDown && !DpadDownButtonWasDown) {
            P = Math.max(0, P - stepsizes[stepIndex]);
            pidfChanged = true;
        }
        DpadDownButtonWasDown = DpadDownButtonIsDown;

        // Only update PIDF if changed
        if (pidfChanged) {
            applyPIDF();
            resetStats();
        }

        if (motorEnabled) {
            FlywheelRight.setVelocity(curTargetVelocity);
        } else {
            FlywheelRight.setPower(0);
        }

        double curVelocity =
                FlywheelRight.getVelocity();

        double error =
                curTargetVelocity - curVelocity;

        // Ignore first second after a tuning change
        if (motorEnabled &&
                getRuntime() - statsStartTime >= 1.0) {

            totalAbsoluteError += Math.abs(error);
            errorSamples++;
        }
        double averageError =
                errorSamples > 0
                        ? totalAbsoluteError / errorSamples
                        : 0;

        boolean atTarget =
                motorEnabled &&
                        Math.abs(error) <= velocityTolerance;

        // TELEMETRY
        telemetry.addData("Motor", "RIGHT");
        telemetry.addData("Motor Enabled", motorEnabled);
        telemetry.addLine("---------------------------");
        telemetry.addData("Target Velocity",
                "%.2f", curTargetVelocity);
        telemetry.addData("Current Velocity",
                "%.2f", curVelocity);
        telemetry.addData("Error",
                "%.2f", error);
        if (errorSamples > 0) {
            telemetry.addData("Average Absolute Error",
                    "%.2f", averageError);
        } else {
            telemetry.addData("Average Absolute Error", "N/A");
        }
        telemetry.addData("At Target", atTarget);
        telemetry.addLine("---------------------------");
        telemetry.addData("Tuning P Value",
                "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F Value",
                "%.4f (D-Pad R/L)", F);
        telemetry.addData("Step Size",
                "%.4f (B Button)", stepsizes[stepIndex]);
        telemetry.addLine("---------------------------");
        telemetry.addLine("Y = Change Velocity");
        telemetry.addLine("B = Change Step Size");
        telemetry.addLine("A = Start/Stop Motor");
        telemetry.update();
    }
}