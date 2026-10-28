package org.firstinspires.ftc.teamcode.Robot2nd;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class SecondRobotTeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor LeftMotor = hardwareMap.dcMotor.get("LeftMotor");
        DcMotor RightMotor = hardwareMap.dcMotor.get("RightMotor");
        DcMotor IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        DcMotor FlywheelMotor = hardwareMap.dcMotor.get("FlywheelMotor");
        DcMotor TransferMotor = hardwareMap.dcMotor.get("TransferMotor");
        double MaxSpeed = 1.0;

        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Variables to handle single button presses (debouncing)
        boolean currentDpadUp = false;
        boolean lastDpadUp = false;
        boolean currentDpadDown = false;
        boolean lastDpadDown = false;

        // --- INITIALISATION LOOP ---
        // This runs repeatedly after you hit INIT, but before you hit PLAY
        while (!isStarted() && !isStopRequested()) {
            currentDpadUp = gamepad1.dpad_up;
            currentDpadDown = gamepad1.dpad_down;

            // Increase speed on single tap of D-pad Up
            if (currentDpadUp && !lastDpadUp) {
                MaxSpeed += 0.1;
            }
            // Decrease speed on single tap of D-pad Down
            if (currentDpadDown && !lastDpadDown) {
                MaxSpeed -= 0.1;
            }

            // Save states for next loop iteration
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // Keep MaxSpeed between a safe limits (0.1 and 1.0)
            MaxSpeed = Math.max(0.1, Math.min(1.0, MaxSpeed));

            // Show current target speed on the Driver Station
            telemetry.addData("Status", "INITIALISED - Press PLAY to start");
            telemetry.addData("Adjust Max Speed (D-Pad Up/Down)", "%.1f", MaxSpeed);
            telemetry.update();

            sleep(10); // Small pause to give the CPU a brief rest
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Max Speed", MaxSpeed);
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double LeftPower = (y + x) / denominator * MaxSpeed;
            double RightPower = (y - x) / denominator * MaxSpeed;

            LeftMotor.setPower(LeftPower);
            RightMotor.setPower(RightPower);
            IntakeMotor.setPower(gamepad1.right_trigger);
            TransferMotor.setPower(gamepad1.left_trigger);

            if (gamepad1.a) {
                FlywheelMotor.setPower(1);
            } else if (gamepad1.x) {
                FlywheelMotor.setPower(0.5);
            } else if (gamepad1.y) {
                FlywheelMotor.setPower(0.25);
            }

            telemetry.update();
        }
    }
}