package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpMode extends GamepadOpMode {
    private final Robot robot = new Robot(this);

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        updateGamepads();

//        robot.updateMecanumRobotDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.options) {
            robot.imu.resetYaw();
        }

        robot.updateMecanumFieldDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        double armCommand = gamepad2.left_stick_y * Robot.ARM_MAX_POWER;
        if (armCommand == 0) {
            // Prevent arm from moving when it should not be moving, and limiting the force applied to hopefully not get shock loads idk.
            robot.armMotor.setPower(Range.clip(0 - robot.armMotor.getVelocity() * 0.0001, -0.3, 0.3));
        } else {
            robot.armMotor.setPower(armCommand);
        }

        // TODO: arm current warning
        // TODO: lift limits
        robot.liftMotor.setPower(gamepad2.right_stick_y * Robot.LIFT_MAX_POWER);
        telemetry.addData("Status", "Running");
        if (currentGamepad2.cross && !previousGamepad1.cross) {
            robot.toggleClaw();
        }
        telemetry.addData("Claw State", robot.clawState);
        telemetry.addData("Lift Position", robot.liftMotor.getCurrentPosition());
        telemetry.addData("Arm Position", robot.armMotor.getCurrentPosition());
        // TODO: telemetry
    }
}
